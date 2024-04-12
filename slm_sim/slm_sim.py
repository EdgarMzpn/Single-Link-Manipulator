#!/usr/bin/env python
import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import JointState

class SLM_Sim(Node):
    def __init__(self):
        super().__init__('SLM_Sim')

        # Setup Variables to be used
        self.k = 0.01
        self.m = 0.75
        self.l = 0.36
        self.a = self.l/2
        self.g = 9.8
        self.J = 4/3 * (self.m * self.a**2)
        self.Tau = 0.0
        self.x1 = 0.0
        self.x2 = 0.0
        self.dt = 0.0
        self.position = JointState()
        self.header = Header()

        # Define Subscribers
        self.sub = self.create_subscription(Float32, 'tau', self.callbackTau, 10)

        # Define Publishers
        self.arm_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tau_pub = self.create_publisher(Float32, "tau", 10)
        self.get_logger().info("The SLM_Sim code is running")
        time_period = 0.1 #seconds
        # Start time
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(time_period, self.sim)
        
    # Define the callback functions
    def callbackTau(self, msg):
        self.Tau = msg.data
        print("I'm being called")

    # Wrap to pi function
    def wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi
    
    # Run pendulum simulation
    def sim(self):
        # Get the current time
        current_time = self.get_clock().now()
        
        # Calculate the duration between current_time and start_time
        duration = current_time - self.start_time

        # Convert the duration to a float value (in seconds)
        self.dt = duration.nanoseconds * 1e-9
        
        # SLM governing equation
        self.x1 += self.x2 * self.dt

        x2_dot = (1/(self.J+self.m*self.a**2)) * (-self.m*self.g*self.a*np.cos(self.x1) - self.k*self.x2 + self.Tau)
        self.x2 += x2_dot*self.dt
        
        print("x1 = ", self.x1)
        print("tau = ", self.Tau)

        self.position.header.stamp = self.get_clock().now().to_msg()
        self.position.name = ["joint2"]
        self.position.position = [self.x1]
        self.position.velocity = [self.x2]
        self.arm_pub.publish(self.position)

        if self.Tau > 0:
            self.tau_pub.publish(Float32(data = 0.0))

        self.start_time = self.get_clock().now()
        time.sleep(0.1)


# Initialize the Node
def main(args=None):
    rclpy.init(args=args)
    slm_sim = SLM_Sim()
    rclpy.spin(slm_sim)
    slm_sim.destroy_node()
    rclpy.shutdown
    
if __name__ == '__main__':
    main()