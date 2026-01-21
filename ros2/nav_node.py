import rclpy
from rclpy.node import Node
import torch
from std_msgs.msg import Float32MultiArray

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')

        self.psi_v = None
        self.meta = None

        self.create_subscription(
            Float32MultiArray, '/psi_vision', self.cb_vision, 10)

        self.create_subscription(
            Float32MultiArray, '/meta_correction', self.cb_meta, 10)

        self.pub = self.create_publisher(
            Float32MultiArray, '/psi_nav', 10)

        self.timer = self.create_timer(0.05, self.step)

    def cb_vision(self, msg):
        self.psi_v = torch.tensor(msg.data)

    def cb_meta(self, msg):
        self.meta = torch.tensor(msg.data)

    def step(self):
        if self.psi_v is None:
            return

        # baseline nav embedding
        psi_nav = nav_model(self.psi_v)

        # meta-correction
        if self.meta is not None:
            psi_nav = psi_nav + self.meta

        out = Float32MultiArray()
        out.data = psi_nav.detach().cpu().tolist()
        self.pub.publish(out)


def nav_model(psi_v):
    # placeholder (MLP / policy)
    return psi_v.clone()


def main():
    rclpy.init()
    node = NavNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
