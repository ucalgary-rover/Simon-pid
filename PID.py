import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

# topic rover_state: [setpoint, previous_error, current_state] input
# topic rover_command: [control_effort] output


class PIDController(Node):
    def __init__(self):
        super().__init__("pid_controller")
        self.publisher = self.create_publisher(
            Float64, "rover_command", 10
        )  # Publish to 'rover_command' topic
        self.subscription = self.create_subscription(
            Float64MultiArray, "rover_state", self.listener_callback, 10
        )  # Subscribe to 'rover_state' topic
        self.kp = 1.0  # Proportional gain
        self.ki = 1.0  # Integral gain
        self.kd = 1.0  # Derivative gain
        self.integral = 0.0  # Integral error

    def listener_callback(self, msg):
        self.setpoint = msg.data[0]  # Set desired state
        self.previous_error = msg.data[1]  # Set previous error
        error = self.setpoint - msg.data[2]  # Calculate error
        self.integral += error  # Accumulate error
        derivative = error - self.previous_error  # Calculate rate of change of error
        output = (
            self.kp * error + self.ki * self.integral + self.kd * derivative
        )  # PID output
        self.publisher.publish(Float64(data=output))  # Publish control effort
        self.previous_error = error  # Update previous error


def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
