import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
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
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.result_pub = self.create_publisher(Bool, '/checkpoint_done', 10)

        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.srv_save = self.create_service(Checkpoints, '/omnicare/checkpoints/save_checkpoint', self.save_callback)
        self.srv_start = self.create_service(Checkpoints, '/omnicare/checkpoints/start', self.start_callback)
        self.srv_cancel = self.create_service(Trigger, '/omnicare/checkpoints/cancel', self.cancel_callback)

        self.package_share_directory = get_package_share_directory('navigation_pkg')
        self.get_logger().info('Serviço /checkpoints pronto!')

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.pose.pose.position)
        mcl_p = msg.pose.pose.position
        mcl_q = msg.pose.pose.orientation

        self.p_x = mcl_p.x
        self.p_y = mcl_p.y

        self.q_x = mcl_q.x        
        self.q_y = mcl_q.y
        self.q_z = mcl_q.z
        self.q_w = mcl_q.w


        self.get_logger().info(f'I heard posX: "{self.p_x}", posY: {self.p_y}')

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

            poses = []
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
                poses.append(p)


            # Call the action server to follow waypoints
            self.get_logger().info(f"Following waypoints: {len(poses)}")
            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = poses
            self._action_client.wait_for_server()

            # Send the goal to the action server and create feedback callback
            self.goal_handle = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_cb)
            
            # Set up callbacks for goal response
            self.goal_handle.add_done_callback(self.goal_response_cb)


            response.success = True
            response.message = "Success"
            return response
        
        except FileNotFoundError as e:
            self.get_logger().error (f"{e}")
            response.success = False
            response.message = f"{e}"
            return response
        
    
    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Present Checkpoint: {feedback.current_waypoint}")

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Follow Waypoints rejected!")
            return

        self.get_logger().info("Follow Waypoints accepted!")

        get_result_future = goal_handle.get_result_async()

        get_result_future.add_done_callback(self.get_result_cb)


    #     get_result_future.add_done_callback(self.action_result_cb)
        
    def get_result_cb(self, future):
        result = future.result().result
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