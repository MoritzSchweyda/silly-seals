#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""
import rclpy
from hippo_msgs.msg import ActuatorSetpoint, DepthStamped, Float64Stamped
from rclpy.node import Node


class DepthControlNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_controller')

        self.current_setpoint = 0.0
        self.current_depth = 0.0
        self.last_time = self.get_clock().now()
        self.integral = 0.0
        self.last_error = 0.0

        self.thrust_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='thrust_setpoint',
                                                qos_profile=1)

        self.setpoint_sub = self.create_subscription(msg_type=Float64Stamped,
                                                     topic='depth_setpoint',
                                                     callback=self.on_setpoint,
                                                     qos_profile=1)
        
        self.depth_error_pub = self.create_publisher(msg_type=Float64Stamped,
                                                      topic='depth_error',
                                                      qos_profile=1)
        
        self.p_gain_pub = self.create_publisher(msg_type=Float64Stamped,
                                                      topic='p_gain',
                                                      qos_profile=1)
        
        self.i_gain_pub = self.create_publisher(msg_type=Float64Stamped,
                                                      topic='i_gain',
                                                      qos_profile=1)
        
        self.d_gain_pub = self.create_publisher(msg_type=Float64Stamped,
                                                      topic='d_gain',
                                                      qos_profile=1)

        self.depth_sub = self.create_subscription(msg_type=DepthStamped,
                                                  topic='depth',
                                                  callback=self.on_depth,
                                                  qos_profile=1)

    def on_setpoint(self, setpoint_msg: Float64Stamped):
        # We received a new setpoint! Let's save it, so that we can use it as
        # soon as we receive new depth data.
        self.current_setpoint = setpoint_msg.data

    def on_depth(self, depth_msg: DepthStamped):
        # We received a new depth message! Now we can get to action!
        current_depth = depth_msg.depth

        self.get_logger().info(
            f"Hi! I'm your controller running. "
            f'I received a depth of {current_depth} m.',
            throttle_duration_sec=1)

        thrust = self.compute_control_output(current_depth)
        # either set the timestamp to the current time or set it to the
        # stamp of `depth_msg` because the control output corresponds to this
        # point in time. Both choices are meaningful.
        # option 1:
        # now = self.get_clock().now()
        # option 2:
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)

    def publish_vertical_thrust(self, thrust: float,
                                timestamp: rclpy.time.Time) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    def compute_control_output(self, current_depth: float) -> float:
        # TODO: Apply the PID control
        msg_e = Float64Stamped() 
        msg_p = Float64Stamped()
        msg_i = Float64Stamped()
        msg_d = Float64Stamped()

        error_p = self.current_setpoint - current_depth

        msg_e.data = error_p
        self.depth_error_pub.publish(msg_e)

        k_p = 3.0 #6.0   # proportional_gain
        k_i = 0.08 #0.08  # integral_gain
        k_d = 0.0 #5.5   # derivative_gain


        # Integral
        now = self.get_clock().now()
        delta_t = now.nanoseconds *1e-9 - self.last_time.nanoseconds *1e-9
        self.last_time = now

        self.integral += delta_t * error_p 

        # Derivative
        error_d = (error_p - self.last_error) / delta_t
        self.last_error = error_p

        # thrust calculation
        if -0.8 <= self.current_setpoint <= -0.1:
            thrust_z = k_p * error_p + k_d * error_d + k_i * self.integral

            msg_p.data = error_p
            self.p_gain_pub.publish(msg_p)

            msg_i.data = self.integral
            self.i_gain_pub.publish(msg_i)

            msg_d.data = error_d
            self.d_gain_pub.publish(msg_d)

        else:
            self.integral = 0.0 
            thrust_z = 0.0 

        return thrust_z
    



def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
