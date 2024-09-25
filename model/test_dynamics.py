"""
Scratch code to test the forward dynamics model.
"""

import matplotlib.pyplot as plt
import numpy as np
import torch as th
import torch.optim as optim

from pypendulum import (
    PendulumParams,
    evaluate_forward_dynamics_double,
    evaluate_forward_dynamics_single,
)

from .train import DynamicsLayer, EnergyLoss, Network


def main():
    params = PendulumParams()
    params.m_b = 1.0
    params.m_1 = 0.25
    params.m_2 = 0.25
    params.l_1 = 0.40
    params.l_2 = 0.21
    params.g = 9.81

    x0_states = th.tensor(
        np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32).reshape([1, -1])
    )

    dt = 0.01

    network = Network(input_dim=4).to("cpu")
    # energy_loss = EnergyLoss(True)
    optimizer = optim.Adam(network.parameters(), lr=0.001)
    # params = th.tensor([1.0, 0.25, 0.25, 0.4, 0.21, 9.81], dtype=th.float32).reshape(
    #     [1, -1]
    # )

    # network.train()
    # for iter_count in range(0, 25000):
    #     optimizer.zero_grad()
    #     outputs = network(x0_states)

    #     x_out = DynamicsLayer.apply(params, dt, outputs, x0_states)

    #     loss, kinetic, potential, theta_1 = energy_loss(params, x_out)

    #     loss.backward()
    #     optimizer.step()

    #     if iter_count % 100 == 0:
    #         print(
    #             f"-- iteration = {iter_count}, loss: {loss.detach().cpu()}, theta: {theta_1.detach().cpu()}"
    #         )

    # u_values = network.params.detach().numpy()
    # print(u_values.tolist())

    # u_controls = np.zeros([1, 800], dtype=np.float32)

    # x_out, x_D_u, x_D_x0 = evaluate_forward_dynamics_single(
    #     [params] * len(x0_states), dt, u_controls, x0_states
    # )

    # x0_perturb = np.zeros_like(x0_states)
    # x0_perturb[:, 1] = 1.0

    # x_out_pos, _, _ = evaluate_forward_dynamics_single(
    #     [params] * len(x0_states),
    #     dt,
    #     u_controls,
    #     x0_states + x0_perturb * 0.001,
    # )
    # x_out_neg, _, _ = evaluate_forward_dynamics_single(
    #     [params] * len(x0_states),
    #     dt,
    #     u_controls,
    #     x0_states - x0_perturb * 0.001,
    # )

    # d_num = (x_out_pos - x_out_neg) / 0.002

    # import ipdb
    # ipdb.set_trace()


if __name__ == "__main__":
    main()
