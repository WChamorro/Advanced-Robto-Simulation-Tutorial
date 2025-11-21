#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float32

def subscriber_callback(msg):                      # Subscriber callback will be invoked every time when a message arrives to the topic it has subsctibed
    value = msg.data
    print("i've heard: ",value)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('python_subscriber')
    subscriber = node.create_subscription(Float32, '/number', subscriber_callback, 10) 
    node.get_logger().info("Subsciber has been started.")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()