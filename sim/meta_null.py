import numpy as np

def meta_null(psi_stack, alpha=0.3):
    center = psi_stack.mean(axis=0)
    corrections = center - psi_stack
    return alpha * corrections.mean(axis=0)
