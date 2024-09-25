"""
Visualize some things.
"""

import matplotlib.pyplot as plt
import numpy as np
import torch as th

from .train import DynamicsLayer, EnergyLoss, Network


def main():
    window_len = 6000
    step_size = 0.001

    u_controls = th.zeros((1, window_len), dtype=th.float32, requires_grad=True)
    x0_initial = th.zeros((1, 4), dtype=u_controls.dtype)
    params = th.tensor(
        [10.0, 0.25, 0.25, 0.15, 0.15, 9.81], dtype=u_controls.dtype
    ).reshape((1, -1))

    # set the initial condition
    x0_initial = th.tensor(
        [0.0, np.pi / 2.0 + 0.1, 0.0, 0.0], dtype=th.float32
    ).reshape([1, -1])

    # set the control input to:
    # u_controls = th.ones((1, window_len), dtype=th.float32, requires_grad=True) * 1.0

    network = Network(input_dim=4)

    import ipdb

    ipdb.set_trace()

    x_out = DynamicsLayer.apply(params, step_size, u_controls, x0_initial)

    _, T, V = EnergyLoss.evaluate_single_loss(params=params, x_states=x_out)

    total_energy = (T + V).detach().numpy()

    import ipdb

    ipdb.set_trace()

    x_out_detached = x_out.detach().cpu()

    fig, ((ax0, ax1), (ax2, ax3), (ax4, _)) = plt.subplots(nrows=3, ncols=2)

    # angle and angle derivative
    ax0.plot(x_out_detached[0, :, 1])
    ax0.set_xlabel("Time")
    ax0.set_ylabel("Radians")
    ax0.grid()

    ax1.plot(x_out_detached[0, :, 3])
    ax1.set_xlabel("Time")
    ax1.set_ylabel("Radians / s")
    ax1.grid()

    # position and position derivative
    ax2.plot(x_out_detached[0, :, 0])
    ax2.set_xlabel("Time")
    ax2.set_ylabel("Meters")
    ax2.grid()

    ax3.plot(x_out_detached[0, :, 2])
    ax3.set_xlabel("Time")
    ax3.set_ylabel("Meters / s")
    ax3.grid()

    # plot energy
    ax4.plot(total_energy[0, :])
    ax4.set_xlabel("Time")
    ax4.set_ylabel("Joules")
    ax4.grid()

    plt.show()


if __name__ == "__main__":
    main()
