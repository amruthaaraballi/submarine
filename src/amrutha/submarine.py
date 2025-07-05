import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
import time

class SubmarineNavigator(Node):
    def __init__(self):  # ✅ Fixed here
        super().__init__('submarine_navigator')

        # Service clients
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Publisher for setpoints
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.get_logger().info("SubmarineNavigator node initialized.")

    def set_mode(self, mode: str):
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for set_mode service...")

        request = SetMode.Request()
        request.custom_mode = mode
        future = self.mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f"Mode set to {mode}.")
        else:
            self.get_logger().error("Failed to set mode.")

    def arm(self):
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for arming service...")

        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Submarine armed.")
        else:
            self.get_logger().error("Failed to arm the submarine.")

    def go_to_position(self, x: float, y: float, z: float, duration: float = 5.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0  # No rotation

        self.get_logger().info(f"Sending setpoint to position: x={x}, y={y}, z={z}")
        start = time.time()
        while time.time() - start < duration:
            pose.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(pose)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    navigator = SubmarineNavigator()

    navigator.get_logger().info("Waiting for MAVROS connection...")
    time.sleep(3.0)

    navigator.set_mode("GUIDED")
    time.sleep(1.0)
    navigator.arm()
    time.sleep(2.0)

    # Move to target position (including a dive)
    target_x = 15.0
    target_y = 13.0
    target_z = -12.0  # Down is negative in NED
    navigator.go_to_position(target_x, target_y, target_z, duration=7.0)

    # Wait at destination
    time.sleep(3.0)

    # Return to origin (surface)
    navigator.go_to_position(0.0, 0.0, 0.0, duration=7.0)

    navigator.get_logger().info("Mission complete. Returning to origin.")
    time.sleep(2.0)

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  # ✅ Fixed here
    main()

