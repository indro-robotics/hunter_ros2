#!/usr/bin/python3
import math
import threading
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

vel_msg = Twist()
joy_msg = Twist()

class HunterSEControlNode(Node):
    def __init__(self):
        global joy_msg
        global vel_msg
        super().__init__('hunter_se_control')

        # Declaring variables
        timer_period = 0.01

        #Declasring variables
        self.base_x_size = 0.800
        self.base_y_size = 0.350

        #Initializing messages:
        joy_msg.linear.x = float(0)
        joy_msg.angular.z = float(0)


        self.vel_msg_prev = 0.0
        self.steer_msg_prev = 0.0
        self.steer_pos = np.array([0, 0], float)  # FR, FL
        self.vel = np.array([0, 0, 0, 0], float)  # RR, RL, FL, FR
        self.steer_pos_prev = np.array([0, 0], float)  # FR, FL

        # Creating Publishers
        self.pub_vel_ = self.create_publisher(
            Float64MultiArray, '/hunter_se/forward_velocity_controller/commands', 10)
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        # Creating Subscribers
        self._jointStates_subscription = self.create_subscription(
            JointState,
            '/hunter_se/joint_states',
            self.jointStates_callback,
            10
        )

        # Creating Actions
        self._action_steer_client = ActionClient(
            self, FollowJointTrajectory, '/hunter_se/steering_trajectory_controller/follow_joint_trajectory')

    def theta_out(self, delta_ack):
        """
        Determines the ackermann steering angle of the outer wheel
        :param delta_ack: goal steering angle of virtual center wheel
        :return: ackermann steering angle of outer wheel theta_out
        """
        return float(math.atan((self.base_x_size*math.tan(delta_ack))/(self.base_x_size + 0.5*self.base_y_size*math.tan(delta_ack))))

    def theta_in(self, delta_ack):
        """
        Determines the ackermann steering angle of the inner wheel
        :param delta_ack: goal steering angle of virtual center wheel
        :return: ackermann steering angle of inner wheel theta_out
        """
        return float(math.atan((self.base_x_size*math.tan(delta_ack))/(self.base_x_size - 0.5*self.base_y_size*math.tan(delta_ack))))

    def timer_callback(self):
        global vel_msg
        global joy_msg

        if joy_msg.linear.x != 0 or joy_msg.angular.z != 0:
            self.vel[:] = joy_msg.linear.x
            vel_array = Float64MultiArray(data=self.vel)
            self.pub_vel_.publish(vel_array)
            self.vel[:] = 0
        else:
            self.vel[:] = vel_msg.linear.x
            vel_array = Float64MultiArray(data=self.vel)
            self.pub_vel_.publish(vel_array)
            self.vel[:] = 0

        if joy_msg.linear.x != 0 or joy_msg.angular.z != 0:
            steer_msg = joy_msg.angular.z
        else:
            steer_msg = vel_msg.angular.z

        if np.abs(steer_msg) > 0.650:  # 40 degrees in radians (Steering limit)
            steer_msg = float(np.sign(steer_msg) * 0.65)
        if steer_msg != self.steer_msg_prev:
            self.send_steer_goal(steer_msg,self.steer_pos_prev)

    def jointStates_callback(self, msg):
        global vel_msg
        global joy_msg

        joint_positions = msg.position

        # if joy_msg.linear.x != 0 or joy_msg.angular.z != 0:
        #     steer_msg = joy_msg.angular.z
        # else:
        #     steer_msg = vel_msg.angular.z

        self.steer_pos_prev = [
            float(joint_positions[0]), float(joint_positions[1])] # FR, FL
        if np.abs(np.sum(self.steer_pos_prev)) < 0.04:
            self.steer_pos_prev = [0.0, 0.0]

        # if np.abs(steer_msg) > 0.650:  # 40 degrees in radians (Steering limit)
        #     steer_msg = float(np.sign(steer_msg) * 0.65)
        # if steer_msg != self.steer_msg_prev:
        #     self.send_steer_goal(steer_msg,self.steer_pos_prev)

    def send_steer_goal(self, steer_msg, steer_pos_prev):
        '''
        steering_joints command controller to send joint_trajectory messages to control steering angle
        :requires: joint_states topic and jointStates_callback - returns current door position information \n
        :requires: cmd_vel topic - listens to cmd_vel.angular.z message for steering control
        :returns: nothing - calls on action server
        '''
        goal_msg = FollowJointTrajectory.Goal()

        # Creating timing normalizer for duration of movement
        norm_coeff = np.abs(steer_msg-self.steer_msg_prev) / 0.650
        steer_time = norm_coeff * 0.5  # 0.5 seconds to move from straight to max turn
        self.steer_msg_prev = steer_msg

        # Creating joint controller messages
        goal_msg = FollowJointTrajectory.Goal()
        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = steer_pos_prev

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(
            seconds=0.5, nanoseconds=0).to_msg()

        if steer_msg > 0:
            self.steer_pos = [self.theta_in(
                steer_msg), self.theta_out(steer_msg)]
        if steer_msg < 0:
            self.steer_pos = [self.theta_out(
                steer_msg), self.theta_in(steer_msg)]
        if steer_msg == 0:
            self.steer_pos = [0.0, 0.0]

        point2.positions = self.steer_pos
        points = [point1, point2]

        goal_msg.goal_time_tolerance = Duration(
            seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = [
            'front_right_steer_joint', 'front_left_steer_joint']
        goal_msg.trajectory.points = points
        self._action_steer_client.wait_for_server()
        self._send_steer_goal_future = self._action_steer_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_steer_goal_future.add_done_callback(
            self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

class JoySubscriberNode(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription_ = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10
        )
        self.subscription_

    def listener_callback(self, data):
        global joy_msg
        
        joy_msg.linear.x = data.axes[1]*5
        joy_msg.angular.z = data.axes[3]*0.650

class CmdVelSubscriberNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.cmdVel_subscription_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdVel_callback,
            10)

    def cmdVel_callback(self, data):
        global vel_msg
        vel_msg = data

def main(args=None):
    rclpy.init(args=None)

    hunterseControl = HunterSEControlNode()
    cmdVelSubscriber = CmdVelSubscriberNode()
    joySubscriber = JoySubscriberNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(hunterseControl)
    executor.add_node(cmdVelSubscriber)
    executor.add_node(joySubscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = hunterseControl.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
