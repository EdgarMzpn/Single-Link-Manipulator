#!/usr/bin/env python
import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import JointState

class SLM_Sim(Node):
    """Class representing the SLM (Single Link Manipulator) Simulation Node

    Attributes
    ----------
    k : float
        Spring constant of the pendulum.
    m : float
        Mass of the pendulum.
    l : float
        Length of the pendulum.
    a : float
        Half the length of the pendulum.
    g : float
        Acceleration due to gravity.
    J : float
        Moment of inertia of the pendulum.
    Tau : float
        Torque applied to the pendulum.
    x1 : float
        Current position of the pendulum.
    x2 : float
        Current velocity of the pendulum.
    dt : float
        Time step of the simulation.
    position : sensor_msgs.msg.JointState
        Joint state message for publishing.
    header : std_msgs.msg.Header
        Header message for joint state.
    """
    def __init__(self):
        """Variable initialization
        """
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
        # Start time
        time_period = 0.1 #seconds
        self.timer = self.create_timer(time_period, self.sim)
        self.start_time = self.get_clock().now()
        
    # Define the callback functions
    def callbackTau(self, msg):
        """Callback function for receiving torque messages

        Parameters
        ----------
        msg : std_msgs.msg.Float32
            Tau message
        """
        self.Tau = msg.data

    # Wrap to pi function
    def wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi
    
    # Run pendulum simulation
    def sim(self):
        """Run the pendulum simulation.

        This method simulates the dynamics of the single-link manipulator (pendulum)
        based on its governing equations. It updates the position and velocity of
        the pendulum and publishes the joint state.
        """
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

        self.start_time = current_time
        # time.sleep(0.1)


# Initialize the Node
def main(args=None):
    rclpy.init(args=args)
    slm_sim = SLM_Sim()
    rclpy.spin(slm_sim)
    slm_sim.destroy_node()
    rclpy.shutdown
    
if __name__ == '__main__':
    main()