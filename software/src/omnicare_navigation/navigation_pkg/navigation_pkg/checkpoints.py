import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger
import json
from pathlib import Path
from os import path


# 5th floor
#
# checkpoint 1 > I heard posX: "1.7204484092596528", posY: -0.6794136066512567
# checkpoint 2 > I heard posX: "6.248311608231612", posY: 0.12551270044226645



# 4th floor
#
# checkpoint >  I heard posX: "-0.6128611658595348", posY: 7.135366317795398


class Checkpoints(Node):

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

        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.srv_save = self.create_service(Trigger, '/omnicare/checkpoints/save_checkpoint', self.save_callback)
        self.srv_start = self.create_service(Trigger, '/omnicare/checkpoints/start', self.start_callback)
        self.srv_cancel = self.create_service(Trigger, '/omnicare/checkpoints/cancel', self.cancel_callback)
        self.declare_parameter('checkpoints_file', '/home/llagoeiro/Desktop/FEI/TCC/TCC/software/src/omnicare_navigation/navigation_pkg/config/map/checkpoints/checkpoints.json')


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
            # Goes up two folders
            base_dir = Path(__file__).resolve().parent.parent 
            filename = base_dir / 'config' / 'map' / 'checkpoints' / 'checkpoints.json'            
            dictObj = []
        
            # Check if file exists
            if path.isfile(filename) is False:
                raise Exception("File not found")
        
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

        try:
            checkpoints_file = self.get_parameter('checkpoints_file').get_parameter_value().string_value

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

            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = poses
            self._action_client.wait_for_server()
            self.goal_handle = self._action_client.send_goal_async(goal_msg)

            response.success = True
            response.message = "Success"
            return response
        except FileNotFoundError as e:
            self.get_logger().error (f"{e}")
            response.success = False
            response.message = f"{e}"
            return response

    def cancel_callback(self, request, response):
        self.goal_handle.result().cancel_goal_async()

        response.success = True
        response.message = "Success"
        return response

def main(args=None):
    rclpy.init(args=args)

    action_client = Checkpoints()
    rclpy.spin(action_client)



if __name__ == '__main__':
    main()