from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_srvs.srv import Trigger
from omnicare_msgs.srv import TeleportFloor
import rclpy
from rclpy.node import Node

import math

class FloorTeleport(Node):
    def __init__(self):
        super().__init__('floor_teleport')
        
        # Cria service
        self.srv = self.create_service(TeleportFloor, '/omnicare/teleport_floor', self.teleport_cb)

        # Cliente para Gazebo
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando /gazebo/set_entity_state...')

        # Defina as coordenadas dos andares
        self.floor_coords = {
            5: {'x': 0.0     , 'y': 0.0     , 'z': 0.1     , 'yaw': 0.0      },  # Andar 5
            4: {'x': 1.842989, 'y': 2.640311, 'z': 0.120965, 'yaw': 1.15}   # Andar 4
        }

        self.get_logger().info('Service /omnicare/teleport_floor pronto!')

    def yaw_to_quaternion(self,yaw):
        half_yaw = yaw / 2.0
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(half_yaw),
            'w': math.cos(half_yaw)
        }

    def teleport_cb(self, request, response):
        if request.target_floor not in self.floor_coords:
            response.success = False
            response.message = 'Andar inválido'
            return response

        coords = self.floor_coords[request.target_floor]
        q = self.yaw_to_quaternion(coords['yaw'])
        state = EntityState()
        state.name = 'OmniCare'  # substitua pelo nome real do seu robô no Gazebo
        state.pose.position.x = coords['x']
        state.pose.position.y = coords['y']
        state.pose.position.z = coords['z']
        state.pose.orientation.x = q['x']
        state.pose.orientation.y = q['y']
        state.pose.orientation.z = q['z']
        state.pose.orientation.w = q['w']

        req = SetEntityState.Request()
        req.state = state
        
        try:
            self.cli.call_async(req)
        except rclpy.ServiceException as e:
            print("gazebo/set_entity_state service call failed")

        response.success = True
        response.message = 'Robô teleportado com sucesso.'
        return response

# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = FloorTeleport()
    rclpy.spin(node)

    # Destrói o nó e encerra o rclpy
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  

