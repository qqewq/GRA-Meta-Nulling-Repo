import rclpy
from rclpy.node import Node
import torch
from std_msgs.msg import Float32MultiArray

ALPHA = 0.3   # сила мета-коррекции (0 = baseline)

class MetaNullNode(Node):
    def __init__(self):
        super().__init__('meta_null_node')

        self.psi = {}

        self.create_subscription(
            Float32MultiArray, '/psi_vision',
            lambda m: self.store('vision', m), 10)

        self.create_subscription(
            Float32MultiArray, '/psi_grasp',
            lambda m: self.store('grasp', m), 10)

        self.create_subscription(
            Float32MultiArray, '/psi_nav',
            lambda m: self.store('nav', m), 10)

        self.pub = self.create_publisher(
            Float32MultiArray, '/meta_correction', 10)

        self.timer = self.create_timer(0.05, self.step)  # 20 Hz

    def store(self, key, msg):
        self.psi[key] = torch.tensor(msg.data)

    def step(self):
        if len(self.psi) < 3:
            return

        psi_stack = torch.stack([
            self.psi['vision'],
            self.psi['grasp'],
            self.psi['nav']
        ])  # [3, d]

        center = psi_stack.mean(dim=0)
        corrections = center - psi_stack

        # агрегируем коррекцию
        meta = ALPHA * corrections.mean(dim=0)

        out = Float32MultiArray()
        out.data = meta.detach().cpu().tolist()
        self.pub.publish(out)

        # лог Φ_meta
        phi_meta = ((psi_stack - center) ** 2).mean().item()
        self.get_logger().info(f'Φ_meta={phi_meta:.4f}')


def main():
    rclpy.init()
    node = MetaNullNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
