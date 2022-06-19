import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from std_srvs.srv import Empty

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.command_pubplisher = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.pose_supscriotion = self.create_subscription(Pose, '/turtle1/pose',self.pose_callback,10)
        self.pose = Pose()
        self.rand_goal_service = self.create_service(Empty, '/rand_goal',self.rand_goal_callback)
        self.goal = np.array([2.0, 3.0])
    def timer_callback(self):
        msg = self.control()
        self.command_pubplisher.publish(msg)
    def pose_callback(self, msg):
        self.pose = msg
    def control(self):
        msg = Twist()
        cur_pos = np.array([self.pose.x,self.pose.y])
        dp = self.goal-cur_pos
        e = np.arctan2(dp[1],dp[0])-self.pose.theta
        K = 3.0
        W = K * np.arctan2(np.sin(e),np.cos(e))
        if np.linalg.norm(dp)>0.1:
            V = 1.0
        else:
            V = 0.0
        msg.linear.x = V
        msg.angular.z = W
        return msg
    def rand_goal_callback(self, req,res):
        self.goal = 9 * np.random.rand(2) + 0.5
        print(self.goal)
        return res



def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()



