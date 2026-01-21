import torch
import torch.nn as nn

class DomainNull(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.encoder = nn.Linear(dim, dim)

    def forward(self, x):
        return self.encoder(x)


class MetaProjector(nn.Module):
    def forward(self, psi_list):
        # Simple shared projection (placeholder)
        return torch.stack(psi_list).mean(dim=0)


def meta_foam(psi_list, projector):
    phi = 0.0
    P = projector(psi_list)
    for i in range(len(psi_list)):
        for j in range(len(psi_list)):
            if i != j:
                phi += torch.sum((psi_list[i] * P * psi_list[j])**2)
    return phi


class GRANullModule(nn.Module):
    def __init__(self, dims):
        super().__init__()
        self.locals = nn.ModuleList([DomainNull(d) for d in dims])
        self.meta_projector = MetaProjector()

    def forward(self, inputs):
        psi = [loc(x) for loc, x in zip(self.locals, inputs)]
        phi_meta = meta_foam(psi, self.meta_projector)
        return psi, phi_meta
