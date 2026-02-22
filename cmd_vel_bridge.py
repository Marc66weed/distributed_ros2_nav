import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class Bridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # 1. 聽樹莓派的新頻道 (TwistStamped)
        self.sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel_nav',
            self.callback,
            10)

        # 2. 講給 Gazebo 聽 (Twist)
        # 因為樹莓派移走了，這裡可以直接用 /cmd_vel，不會撞車了！
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        print("翻譯官 V3 已就位：/cmd_vel_nav -> /cmd_vel")

    def callback(self, msg):
        self.pub.publish(msg.twist)

def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
