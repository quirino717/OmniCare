import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from omnicare_msgs.srv import Checkpoints
import json
from pathlib import Path
from os import path
from ament_index_python.packages import get_package_share_directory

# Example service call to save a checkpoint
# ros2 service call /omnicare/checkpoints/save_checkpoint omnicare_msgs/srv/Checkpoints "{floor: 'simulation'}"


class checkpointsService(Node):

    def __init__(self):
        super().__init__('checkpoints')
        self.goal_handle = None
        self.p_x,self.p_y,self.q_x,self.q_y,self.q_z,self.q_w = None,None,None,None,None,None
        self.init_p_x,self.init_p_y = None,None  
        self.tf_p_x,self.tf_p_y = None,None
        self.hasConverged = False
        self.poses = []
        self.package_share_directory = get_package_share_directory('navigation_pkg')
        

        # Pubs
        self.result_pub = self.create_publisher(Bool, '/checkpoint_done', 10)

        # Subs
        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.initial_pose_callback,10,callback_group=ReentrantCallbackGroup())
        self.pose_from_tf_sub = self.create_subscription(PoseStamped,'/pose_from_tf',self.tf_callback,10,callback_group=ReentrantCallbackGroup())
        self.amcl_pose = self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.amcl_callback,10)

        # Services
        self.srv_save = self.create_service(Checkpoints, '/omnicare/checkpoints/save_checkpoint', self.save_callback)
        self.srv_start = self.create_service(Checkpoints, '/omnicare/checkpoints/start', self.start_callback)
        self.srv_cancel = self.create_service(Trigger, '/omnicare/checkpoints/cancel', self.cancel_callback)

        # Action Client
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        self.gate_timer = None
        self._timer_cb = ReentrantCallbackGroup()


        
        self.get_logger().info('Serviço /checkpoints pronto!')
    
    def initial_pose_callback(self, msg):
        init_p = msg.pose.pose.position
        
        if init_p.x is None or init_p.y is None:
            return
        
        self.init_p_x = init_p.x
        self.init_p_y = init_p.y


        self.get_logger().info(f'INI: I heard posX: "{self.p_x}", posY: {self.p_y}')

    def tf_callback(self, msg):
        tf_p = msg.pose.position

        if tf_p.x is None or tf_p.y is None:
            return

        self.tf_p_x = tf_p.x
        self.tf_p_y = tf_p.y

        self.get_logger().debug(f'TF: I heard posX: "{self.p_x}", posY: {self.p_y}')

        # self.get_logger().info(f"The subtraction is: X={abs(self.init_p_x - self.tf_p_x)} and Y={abs(self.init_p_y - self.tf_p_y)}")

    def amcl_callback(self, msg):
        mcl_p = msg.pose.pose.position
        mcl_q = msg.pose.pose.orientation

        self.p_x = mcl_p.x
        self.p_y = mcl_p.y

        self.q_x = mcl_q.x        
        self.q_y = mcl_q.y
        self.q_z = mcl_q.z
        self.q_w = mcl_q.w

    def save_callback(self, request, response):

        try:
            filename = f"{self.package_share_directory}/config/checkpoint/{request.floor}_checkpoints.json"          
            dictObj = []
        
            # Check if file exists
            if path.isfile(filename) is False:
                self.get_logger().error(f"File not found: {filename}")
                response.success = False
        
            # Read JSON file
            with open(filename) as fp:
                dictObj = json.load(fp)

            if self.p_x is None or self.p_y is None or self.q_x is None or self.q_y is None or self.q_z is None or self.q_w is None:
                self.get_logger().info("Waiting for pose data...")
                response.success = False
                response.message = "Waiting for pose data"
                return response

            else:
                # Append new pose data
                new_pose = {
                    "position": {"x": round(self.p_x,2), "y": round(self.p_y,2), "z": 0.0},
                    "orientation": {"x": round(self.q_x,2), "y": round(self.q_y,2), "z": round(self.q_z,2), "w": round(self.q_w,2)}
                }        

                dictObj["poses"].append(new_pose)
                
                self.get_logger().info(str(dictObj))


                with open(filename, 'w') as json_file:
                    json.dump(dictObj, json_file, indent=4)

                # goal_msg = FollowWaypoints.Goal()
                # goal_msg.poses = poses
                # self._action_client.wait_for_server()
                # self.goal_handle = self._action_client.send_goal_async(goal_msg)

                response.success = True
                response.message = "Success"
                return response
        except FileNotFoundError as e:
            self.get_logger().error (f"{e}")
            response.success = False
            response.message = f"{e}"
            return response

    def start_callback(self, request, response):
        self.get_logger().info(f"Received request to start checkpoints for floor: {request.floor}")
        
        try:
            checkpoints_file = f"{self.package_share_directory}/config/checkpoint/{request.floor}_checkpoints.json"
            self.get_logger().info(f"Starting checkpoints from file: {checkpoints_file}")

            f = open(checkpoints_file)
            data = json.load(f)

            for i in data['poses']:
                p = PoseStamped()
                p.header.frame_id = data['frame_id']
                p.pose.position.x = i['position']['x']
                p.pose.position.y = i['position']['y']
                p.pose.position.z = i['position']['z']
                p.pose.orientation.x = i['orientation']['x']
                p.pose.orientation.y = i['orientation']['y']
                p.pose.orientation.z = i['orientation']['z']
                p.pose.orientation.w = i['orientation']['w']
                self.poses.append(p)


            if self.gate_timer is None:
                self.gate_timer = self.create_timer(0.1, self._converged_cb, callback_group=ReentrantCallbackGroup())

            response.success = True
            response.message = "Waypoints enfileirados. Aguardando convergência TF≈SET_POSE para iniciar."
            return response
        
        except FileNotFoundError as e:
            self.get_logger().error (f"{e}")
            response.success = False
            response.message = f"{e}"
            return response
        
    def _converged_cb(self):
        if self.init_p_x is None or self.init_p_y is None or self.tf_p_x is None or self.tf_p_y is None:
            self.get_logger().info("Waiting for initial and TF pose data...")
            return
        
        range = 0.1
        if abs(self.init_p_x - self.tf_p_x) < range and abs(self.init_p_y - self.tf_p_y) < range:
            self._start_follow_waypoints(self.poses)
            self.poses = []
            self.gate_timer.cancel()
            self.gate_timer = None
            self.get_logger().info('Convergência TF≈SET_POSE atingida. Iniciando FollowWaypoints.')

            
    
    def _start_follow_waypoints(self, poses):
        self.get_logger().info(f"Following waypoints: {len(poses)}")
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        self._action_client.wait_for_server()

        # Send the goal to the action server and create feedback callback
        self.goal_handle = self._action_client.send_goal_async(goal_msg,feedback_callback=self._feedback_cb)
        
        # Set up callbacks for goal response
        self.goal_handle.add_done_callback(self._goal_response_cb)

    
    def _feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Present Checkpoint: {feedback.current_waypoint}")

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Follow Waypoints rejected!")
            return

        self.get_logger().info("Follow Waypoints accepted!")

        get_result_future = goal_handle.get_result_async()

        get_result_future.add_done_callback(self._get_result_cb)


        
    def _get_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"Follow Waypoints result: {result.missed_waypoints}")
        if len(result.missed_waypoints) == 0:
            self.get_logger().info("Follow Waypoints completed successfully!")
            self.result_pub.publish(Bool(data=True))



    def cancel_callback(self, request, response):
        self.goal_handle.result().cancel_goal_async()

        response.success = True
        response.message = "Success"
        return response

def main(args=None):
    rclpy.init(args=args)

    action_client = checkpointsService()
    rclpy.spin(action_client)



if __name__ == '__main__':
    main()