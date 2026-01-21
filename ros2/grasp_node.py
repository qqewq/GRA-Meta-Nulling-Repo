class GraspNode(Node):
    def __init__(self):
        self.psi_v = None
        self.sub = self.create_subscription(Psi, "/psi_vision", self.cb, 10)
        self.pub = self.create_publisher(Psi, "/psi_grasp", 10)

    def cb(self, msg):
        self.psi_v = torch.tensor(msg.data)

    def step(self, obs):
        psi = model(obs, self.psi_v)
        self.pub.publish(Psi(data=psi.tolist()))
