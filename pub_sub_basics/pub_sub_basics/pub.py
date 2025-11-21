#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float32

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('python_subscriber')

    publisher = node.create_publisher(Float32,"/number",10)
    print("starting incresing number")
    rate = node.create_rate(1)

    num=0.0

    while rclpy.ok():
        msg = Float32()
        msg.data = num
        publisher.publish(msg)
        num= num + 1.5
        rclpy.spin_once(node)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
