import math
import tf2_ros
import rclpy
from time import sleep
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener,TransformBroadcaster
from example_interfaces.srv import Trigger
import numpy as np

class pid_sim(Node):

    def __init__(self):
        super().__init__('pid_sim')
        # Create timer and tf subscriber
        self.dt = 1.0/30.0
        self._tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self._tf_buffer,self)
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self._to_frame = 'odom'
        self._from_frame = 'chassis'

        # Create goal_pose subscriber
        self.goal_pose_sub = self.create_subscription(PoseStamped, 'goal_pose', self.listener_callback, 10)

        # Create cmd_vel publisher
        self.cmd_vel_pub = self.create_publisher(Twist,'cmd_vel',QoSProfile(depth=10))

        # inputs and initial conditions
        self.l = 0.4 # Offset input for PD control for diffDrive robot according to lecture
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.goal_received = False
        
        # PID Controller Inputs
        self.kp = 0.2
        self.ki = 0.0
        self.kd = 0.2

    def timer_callback(self):
        if not self.goal_received:
            return
        try:
            when = rclpy.time.Time()
            # Get current position
            trans = self._tf_buffer.lookup_transform(self._to_frame, self._from_frame, when, timeout = Duration(seconds = 5.0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            theta = 2.0*np.arctan2(qz,qw)

            v = self.pidControl(x,y,theta)
            vf = v[0]
            omega = v[1]

            # Publish velocity command
            cmd = Twist()
            cmd.linear.x = vf
            cmd.angular.z = omega
            self.cmd_vel_pub.publish(cmd)

        except tf2_ros.LookupException:
            self.get_logger().info('Transform isn\'t available, waiting...')
            sleep(1)
            return

    def listener_callback(self, msg: PoseStamped):
        # From lecture, only need position information, not concerned over heading right now. 2D, so only need x/y
        self.goal_pose_x = msg.pose.position.x
        self.goal_pose_y = msg.pose.position.y
        self.goal_received = True
        
    def pidControl(self,x,y,theta):
        # Add offset l to get xp and yp
        xp = x + self.l*np.cos(theta)
        yp = y + self.l*np.sin(theta)

        # PID Control
        error = np.array([self.goal_pose_x - xp,self.goal_pose_y - yp])
        vel = np.array([(x - self.prev_x)/self.dt, (y - self.prev_y)/self.dt])
        output = self.kp*error - self.kd*vel
        self.prev_x = x
        self.prev_y = y

        # Create rotation matrix
        ct = np.cos(theta)
        st = np.sin(theta)
        R = np.array([[ct, st],
                        [-st, ct]])

        v = R@output
        vf = v[0]
        omega = v[1]/self.l
        return np.array([vf,omega])
    
def main(args=None):
    rclpy.init(args=args)

    PID_SIM = pid_sim()

    rclpy.spin(PID_SIM)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    PID_SIM.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()