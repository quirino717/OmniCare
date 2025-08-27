import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from omnicare_msgs.action import EnterElevator  

import math, statistics
from collections import deque

class EnterElevatorServer(Node):

    def __init__(self):
        super().__init__('enter_elevator_server')


        self.range_left, self.range_right, self.range_front = float('inf'), float('inf'), float('inf')
        self.alignment_history = deque(maxlen=5)

        simulation = self.declare_parameter('simulation',False).get_parameter_value().bool_value
        # simulation = self.get_parameter('simulation').get_parameter_value().bool_value

        self.get_logger().info(f'Simulation mode: {simulation}')
        if simulation:
            self.angle_left  =  45
            self.angle_right = -45
            self.angle_front =   0

            self.vel_left  =   -0.05
            self.vel_right =    0.05
        else:
            self.angle_left  = -135
            self.angle_right =  135
            self.angle_front =  180

            self.vel_left  =   0.05
            self.vel_right =  -0.05




        # Subs
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )


        # Pub
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Action
        self._action_server = ActionServer(
            self,
            EnterElevator,
            'omnicare/elevator/enter_elevator',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Action /enter_elevator pronto!')

    def is_aligned(self,range_left, range_right, tolerance=0.2):
        """
        Verifica se o robô está alinhado com o elevador comparando simetria dos lados.
        """
        if range_left == float('inf') or range_right == float('inf'):
            return False  # Um dos lados está sem leitura, não confiável

        diff = abs(range_left - range_right)
        self.get_logger().info(f"Verificando alinhamento: Esquerda: {range_left:.2f}, Direita: {range_right:.2f}, Diferença: {diff:.2f}")
        return diff <= tolerance

    def get_scan_index(self,angle_rad, scan_msg):
        """
        Converte um ângulo (em rad) no índice do ranges[].
        """
        index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
        index = max(0, min(index, len(scan_msg.ranges) - 1))  # clamp
        return index
    

    def get_median_range_in_sector(self, scan_msg, start_angle_deg, end_angle_deg):
        start_angle = math.radians(start_angle_deg)
        end_angle = math.radians(end_angle_deg)

        start_idx = self.get_scan_index(start_angle, scan_msg)
        end_idx = self.get_scan_index(end_angle, scan_msg)

        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx

        ranges = scan_msg.ranges[start_idx:end_idx+1]
        valid = [r for r in ranges if 0.05 < r < scan_msg.range_max]

        return statistics.median(valid) if valid else float('inf')
    
    def get_average_range_in_sector(self, scan_msg, start_angle_deg, end_angle_deg):
        start_angle = math.radians(start_angle_deg)
        end_angle = math.radians(end_angle_deg)

        start_idx = self.get_scan_index(start_angle, scan_msg)
        end_idx = self.get_scan_index(end_angle, scan_msg)

        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx  # garante ordem crescente

        ranges = scan_msg.ranges[start_idx:end_idx+1]
        valid_ranges = [r for r in ranges if r > 0.05 and r < scan_msg.range_max]

        if not valid_ranges:
            return float('inf')  # não dá para confiar

        return sum(valid_ranges) / len(valid_ranges)

    def laser_callback(self, msg):
        # self.range_left = self.get_average_range_in_sector(msg, self.angle_left - 5, self.angle_left + 5)
        # self.range_right = self.get_average_range_in_sector(msg, self.angle_right - 5, self.angle_right + 5)
        # self.range_front = self.get_average_range_in_sector(msg, self.angle_front - 2, self.angle_front + 2)
        self.range_left  = self.get_median_range_in_sector(msg, self.angle_left  - 10, self.angle_left  + 10)
        self.range_right = self.get_median_range_in_sector(msg, self.angle_right - 10, self.angle_right + 10)
        self.range_front = self.get_median_range_in_sector(msg, self.angle_front -  4, self.angle_front  + 4)

        # self.get_logger().info(
        #     f"Frente: {self.range_front:.2f} m | Esq (+45°): {self.range_left:.2f} m | Dir (-45°): {self.range_right:.2f} m"
        # )
        self.is_aligned(self.range_left,self.range_right,0.1)



    def goal_callback(self, goal_request):
        self.get_logger().info('Recebido pedido de entrar no elevador')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelamento recebido')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executando ação de entrar no elevador...')

        feedback_msg = EnterElevator.Feedback()
        result = EnterElevator.Result()

        rate = self.create_rate(10) #10 Hz
        entering_elevator = False
        while rclpy.ok():            
            # self.get_logger().info('Iterando até entrar no elevador....')
            # self.get_logger().info(
            #     f"Frente: {self.range_front:.2f} m | Esq (+45°): {self.range_left:.2f} m | Dir (-45°): {self.range_right:.2f} m"
            # )

            twist_msg = Twist()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Ação cancelada')

                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist_msg)
                return EnterElevator.Result(success=False, message='Cancelado')
            
            
            if self.is_aligned(self.range_left, self.range_right) or entering_elevator:
                # self.get_logger().info('Robô alinhado com o elevador, entrando...')
                feedback_msg.robot_feedback = 'Robô já está alinhado com o elevador.'
                goal_handle.publish_feedback(feedback_msg)

                entering_elevator = True
                
                # Robô já está alinhado, avança para entrar no elevador
                if self.range_front > 0.5:
                    twist_msg.linear.x = 10.0  
                    self.cmd_vel_publisher.publish(twist_msg)
                
                # Entrou no elevador
                else:
                    twist_msg.linear.x = 0.0
                    self.cmd_vel_publisher.publish(twist_msg)
                    feedback_msg.robot_feedback = 'Entrou no elevador!' 
                    
                    goal_handle.succeed()
                    result.success = True
                    result.message = 'Elevador entrado com sucesso!'
                    return result
    

            elif not entering_elevator and not self.is_aligned(self.range_left, self.range_right):
                # self.get_logger().info('Robô não está alinhado com o elevador, ajustando...')
                # Ajuste de alinhamento
                if self.range_left < self.range_right:
                    self.get_logger().info('ESQUERDA...')
                    twist_msg.angular.z = self.vel_left
                else:
                    self.get_logger().info('DIREITA...')
                    twist_msg.angular.z = self.vel_right
                    
                # self.cmd_vel_publisher.publish(twist_msg)
                feedback_msg.robot_feedback = 'Ajustando alinhamento...'

            self.cmd_vel_publisher.publish(twist_msg)
            goal_handle.publish_feedback(feedback_msg)

            rate.sleep()
            

        


def main(args=None):
    rclpy.init(args=args)
    node = EnterElevatorServer()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
