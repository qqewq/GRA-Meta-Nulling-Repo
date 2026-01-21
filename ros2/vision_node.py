class VisionNode(Node):
    def __init__(self):
        self.pub = self.create_publisher(Psi, "/psi_vision", 10)

    def callback(self, image):
        psi = model(image)
        msg = Psi()
        msg.data = psi.detach().cpu().tolist()
        self.pub.publish(msg)
