import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from std_msgs.msg import String

class NavigationGoalActionClient(Node):
    def __init__(self):
        super().__init__('navigation_goal_action_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscription = self.create_subscription(PoseStamped, 'project',self.send_goal, 10)
        self.subscription
        self.cancel_subscription = self.create_subscription(String, 'cancel2',self.cancel_goal, 10)
        self.cancel_subscription
        self.goal_active = False
        
    def cancel_goal(self, msg):
        if msg:
            self.goal_handle.cancel_goal_async()
            self.goal_active = True
        
    def send_goal(self, msg):
        if not self.goal_active:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.pose.position.x = msg.pose.position.x
            goal_msg.pose.pose.position.y = msg.pose.position.y
            goal_msg.pose.pose.orientation.z = msg.pose.orientation.z
            goal_msg.pose.pose.orientation.w = msg.pose.orientation.w
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            self.action_client.wait_for_server()
            self.goal_active = True
            self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_active = False
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.goal_active = False 

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(feedback)

def main(args=None):
    rclpy.init(args=args)
    navigation_goal_action_client = NavigationGoalActionClient()
    rclpy.spin(navigation_goal_action_client)



if __name__ == '__main__':
    main()
