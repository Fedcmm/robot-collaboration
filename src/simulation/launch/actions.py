import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ros2_data.msg import JointPose
from ros2_data.action import MoveJ


class MoveJclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveJ_client')
        self._action_client = ActionClient(self, MoveJ, 'MoveJ')
        # 2. Wait for MoveJ server to be available:
        print ("Waiting for MoveJ action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveJ ACTION SERVER detected.")
    
    def send_goal(self, GoalJP, JointSPEED):
        # 1. Assign variables:
        goal_msg = MoveJ.Goal()
        goal_msg.goal = GoalJP
        goal_msg.speed = JointSPEED
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveJ ACTION CALL finished.") 

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveJ ACTION CALL.


def main(args=None):
    global RES

    rclpy.init(args=args)

    action_client = MoveJclient()

    goal_pose = JointPose()
    goal_pose.joint1 = 21.0
    goal_pose.joint2 = 30.0
    goal_pose.joint3 = 40.0
    goal_pose.joint4 = 20.0
    goal_pose.joint5 = 20.0
    goal_pose.joint6 = 20.0
    action_client.send_goal(goal_pose, 1.0)
    rclpy.spin(action_client)

    print(f'Result of MoveJ is {RES}')

if __name__ == '__main__':
    main()
