"""
Visualize some things.
"""

import matplotlib.pyplot as plt
import numpy as np
import torch as th

from .train import DynamicsLayer


def main():
    window_len = 200
    u_controls = th.zeros((1, window_len), dtype=th.float32, requires_grad=True)
    x0_initial = th.zeros((1, 4), dtype=u_controls.dtype)
    params = th.tensor(
        [10.0, 0.25, 0.25, 0.15, 0.15, 9.81], dtype=u_controls.dtype
    ).reshape((1, -1))

    x_out = DynamicsLayer.apply(params, 0.01, u_controls, x0_initial)

    x_out_detached = x_out.detach().cpu()

    import ipdb

    ipdb.set_trace()


if __name__ == "__main__":
    main()
