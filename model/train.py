"""
Train a model to control double pendulum.
"""

import argparse
import json
import typing as T
from pathlib import Path

import numpy as np
import pytorch_lightning as pl
import torch as th
import torch.utils.data as td
from pytorch_lightning.callbacks import ModelCheckpoint
from pytorch_lightning.loggers import TensorBoardLogger
from torch import nn

from pypendulum import (
    PendulumParams,
    evaluate_forward_dynamics_double,
    evaluate_forward_dynamics_single,
)

SCRIPT_PATH = Path(__file__).parent.absolute()


class DynamicsLayer(th.autograd.Function):
    """
    Custom layer that implements the forward dynamics of the model.
    """

    @staticmethod
    def forward(
        ctx, params: th.Tensor, dt: float, u_controls: th.Tensor, x0_states: th.Tensor
    ):
        B, param_dim = params.shape
        assert param_dim == 6, "Should be 6 params"
        assert len(u_controls) == B and len(x0_states) == B, "Batch dim mismatched"

        # Convert params so we can pass to C++:
        params_structs = []
        for row in params:
            params_structs.append(PendulumParams(*row))

        if x0_states.shape[-1] == 4:
            x_out, x_D_u, x_D_x0 = evaluate_forward_dynamics_single(
                params_structs, dt, u_controls.detach().cpu(), x0_states
            )
        else:
            x_out, x_D_u = evaluate_forward_dynamics_double(
                params_structs, dt, u_controls.detach().cpu(), x0_states
            )

        # Save the derivatives for the backward pass:
        ctx.save_for_backward(
            th.tensor(x_D_u, dtype=th.float32), th.tensor(x_D_x0, dtype=th.float32)
        )
        return th.tensor(x_out, dtype=th.float32)

    @staticmethod
    def backward(ctx, loss_D_x: th.Tensor):
        # `u_controls` had shape (B, N).
        # `loss_D_x` will have shape (B, N, D), the same as `x_out`.
        # We need to output something with the same shape as `u_controls`: (B, N)
        (x_D_u, x_D_x0) = ctx.saved_tensors

        # x_D_u is ordered over: [batch, x_out, u_control, D]
        # We swap it such that the last two dims are: [..., x_out, D]
        x_D_u = th.permute(x_D_u, (0, 2, 1, 3))

        # Compute grad_u, which will be [B, N] dimension.
        # Insert a new dimension and multiply loss_D_x with x_D_u.
        # (B, 1, N, D) * (B, N, N, D) --> (B, N, N, D)
        # This broadcasts over the `u` dimension while multiplying over the (x_out, D) dims.
        loss_D_u = th.multiply(loss_D_x[:, None, :, :], x_D_u)

        # For each `u`, sum over all `xs` and complete the 1xD * Dx1 chain rule of:
        #   loss_D_x[i] * x[i]_D_u
        loss_D_u = th.sum(loss_D_u, axis=[-2, -1])

        # Compute the (B, N, D) chain rule:
        #   loss_D_x[i] * x[i]_D_x0
        #   (B, N, 1, D) @ (B, N, D, D) --> (B, N, 1, D)
        loss_D_x0 = th.matmul(loss_D_x[:, :, None, :], x_D_x0)

        # Sum: (B, N, 1, D) --> (B, D)
        loss_D_x0 = th.sum(loss_D_x0, axis=[1, 2])

        return None, None, loss_D_u, loss_D_x0


class EnergyLoss(nn.Module):
    """Evaluate loss based on Lagrangian of the pendulum."""

    def __init__(self, is_single: bool, weighting_curve: str):
        super().__init__()
        self.is_single = is_single
        self.weighting_curve = weighting_curve

    def get_time_weighting_curve(self, N: int) -> th.Tensor:
        if self.weighting_curve == "smoothstep":
            lin_spaced = th.linspace(0.0, 1.0, N, dtype=th.float32)[None, ...]
            weighting = 3 * (lin_spaced**2) - 2 * (lin_spaced**3)
        elif self.weighting_curve == "last":
            weighting = th.zeros(size=(1, N), dtype=th.float32)
            weighting[:, -1] = 1.0
        else:
            raise RuntimeError(f"Invalid weighting function: {self.weighting_curve}")

        weighting = weighting / th.sum(weighting)
        return weighting

    def evaluate_single_loss(
        self, params: th.Tensor, x_states: th.Tensor, u_controls: th.Tensor
    ) -> T.Tuple[th.Tensor, T.Dict]:
        """Energy loss for the single pendulum cart-pole system."""
        assert x_states.shape[-1] == 4, "Should be a 4 dimensional state"

        params = params[:, None, :]
        m_b, m_1, l_1, g = (
            params[..., 0],
            params[..., 1],
            params[..., 3],
            params[..., 5],
        )

        b_x, th_1, b_x_dot, th_1_dot = (
            x_states[..., 0],
            x_states[..., 1],
            x_states[..., 2],
            x_states[..., 3],
        )

        # Compute kinetic energy of the system:
        b_dot = th.dstack([b_x_dot, th.zeros_like(b_x)])

        p1_dot = b_dot + th.dstack(
            [-th.sin(th_1) * l_1 * th_1_dot, th.cos(th_1) * l_1 * th_1_dot]
        )
        T = 0.5 * (m_b * th.sum(b_dot**2, axis=-1) + m_1 * th.sum(p1_dot**2, axis=-1))

        # Compute potential energy of the system:
        V = m_1 * g * l_1 * th.sin(th_1)

        # Loss is weighted higher as time increases:
        _, N = T.shape
        weighting = self.get_time_weighting_curve(N=N).to(dtype=th_1.dtype)

        control_derivative = th.diff(u_controls, axis=-1)
        controls_loss = th.sum(th.abs(control_derivative))

        # translation_loss = th.sum(th.abs(b_x) * weighting)

        translation_loss = th.sum(th.mean(b_x, axis=-1))

        loss = (
            th.sum((T - V) * weighting) + translation_loss * 0.0 + controls_loss * 0.0
        )

        max_base_speed, _ = th.max(b_x_dot, axis=1)
        values_out = dict(
            weighted_kinetic=th.mean(th.sum(T * weighting, axis=-1)),
            weighted_potential=th.mean(th.sum(V * weighting, axis=-1)),
            weighted_abs_translation=th.mean(th.sum(th.abs(b_x) * weighting, axis=-1)),
            final_kinetic=th.mean(T[:, -1]),
            final_potential=th.mean(V[:, -1]),
            max_b_x_dot=th.mean(max_base_speed),
            loss=loss,
        )
        return (loss, values_out)

    @staticmethod
    def evaluate_double_loss(params: th.Tensor, x_states: th.Tensor):
        """Energy loss for the double pendulum cart-pole system."""
        assert x_states.shape[-1] == 6, "Should be a 6 dimensional state"

        params = params[:, None, :]
        m_b, m_1, m_2, l_1, l_2, g = (
            params[..., 0],
            params[..., 1],
            params[..., 2],
            params[..., 3],
            params[..., 4],
            params[..., 5],
        )

        b_x, th_1, th_2, b_x_dot, th_1_dot, th_2_dot = (
            x_states[..., 0],
            x_states[..., 1],
            x_states[..., 2],
            x_states[..., 3],
            x_states[..., 4],
            x_states[..., 5],
        )

        # Compute kinetic energy of the system:
        b_dot = th.dstack([b_x_dot, th.zeros_like(b_x)])

        p1_dot = b_dot + th.dstack(
            [-th.sin(th_1) * l_1 * th_1_dot, th.cos(th_1) * l_1 * th_1_dot]
        )
        p2_dot = p1_dot + th.dstack(
            [-th.sin(th_2) * l_2 * th_2_dot, th.cos(th_2) * l_2 * th_2_dot]
        )
        T = 0.5 * (
            m_b * th.sum(b_dot**2, axis=-1)
            + m_1 * th.sum(p1_dot**2, axis=-1)
            + m_2 * th.sum(p2_dot**2, axis=-1)
        )

        # Compute potential energy of the system:
        p1_y = l_1 * th.sin(th_1)
        p2_y = p1_y + l_2 * th.sin(th_2)
        V = m_1 * g * p1_y + m_2 * g * (p1_y + p2_y)

        # Loss is weighted higher as time increases:
        _, N = T.shape
        lin_spaced = th.linspace(0.0, 1.0, N, dtype=T.dtype)[None, ...]
        weighting = 3 * (lin_spaced**2) - 2 * (lin_spaced**3)
        loss = th.mean((T - V) * weighting)

        return (loss, th.mean(T), th.mean(V))

    def forward(self, params: th.Tensor, x_states: th.Tensor, u_controls: th.Tensor):
        if self.is_single:
            return self.evaluate_single_loss(
                params=params, x_states=x_states, u_controls=u_controls
            )
        else:
            return self.evaluate_double_loss(params=params, x_states=x_states)


class Network(nn.Module):
    """Network to control the pendulum."""

    def __init__(self, input_dim: int) -> None:
        super().__init__()
        self.layer0 = nn.Linear(input_dim, 128)
        self.layer1 = nn.Linear(256, 128)
        self.layer2 = nn.Linear(256, 128)
        self.output_layer = nn.Linear(128, 400)

    def forward(self, x_initial_states: th.Tensor) -> th.Tensor:
        l0 = self.layer0(x_initial_states)
        l1 = self.layer1(th.concatenate([nn.functional.tanh(l0), l0], axis=-1))
        l2 = self.layer2(th.concatenate([nn.functional.tanh(l1), l1], axis=-1))
        return self.output_layer(l2)


class ControlNetwork(nn.Module):
    """Network to control the pendulum."""

    def __init__(self, input_dim: int) -> None:
        super().__init__()
        self.layer0 = nn.Linear(input_dim, 32)
        self.layer1 = nn.Linear(32, 32)
        self.layer2 = nn.Linear(32, 32)
        self.output_layer = nn.Linear(32, 1)

    def forward(self, x_initial_states: th.Tensor) -> th.Tensor:
        l0 = self.layer0(x_initial_states)
        l1 = self.layer1(nn.functional.relu(l0))
        l2 = self.layer2(nn.functional.relu(l1))
        result = self.output_layer(l2)
        return result


class Monitor(TensorBoardLogger):
    """Log training loss."""

    def __init__(self, run_dir, log_interval: int = 1):
        self.run_dir = Path(run_dir)
        self.log_interval = log_interval
        self.mode = "train"
        super().__init__(run_dir)

    def add_prefix(self, name: str) -> str:
        return f"{self.mode}/{name}"

    def log_value(self, name: str, value: th.Tensor, step: int):
        if th.is_tensor(value):
            value = value.item()
        self.experiment.add_scalar(self.add_prefix(name), value, step)

    def log_values(self, values: T.Dict[str, th.Tensor], step: int):
        for name, value in values.items():
            self.log_value(name, value, step)


class System(pl.LightningModule):

    def __init__(self, hparams: T.Dict, monitor: T.Optional[Monitor] = None):
        super().__init__()
        self.is_single = hparams["version"] == "single"
        self.model = ControlNetwork(input_dim=4 if self.is_single else 6)
        self.monitor = monitor
        self.energy_loss = EnergyLoss(is_single=self.is_single, weighting_curve="last")
        self.save_hyperparameters(hparams)

    def configure_optimizers(self):
        return th.optim.Adam(self.parameters(), lr=0.001)

    def training_step(self, batch):
        # (1) Query network for trajectory:
        # u_output = self.model(batch["x0_state"])

        # (2) Integrate the dynamics model:
        # x_out = DynamicsLayer.apply(batch["params"], 0.01, u_output, batch["x0_state"])

        dt = 0.01
        window_len = 150
        x = batch["x0_state"]
        control_inputs = []
        x_out = []
        for _ in range(0, window_len):
            u_control = self.model(x)
            x = DynamicsLayer.apply(batch["params"], dt, u_control, x)
            x = th.squeeze(x, axis=1)
            control_inputs.append(u_control)
            x_out.append(x)

        x_stacked = th.stack(x_out, axis=1)
        loss, values = self.energy_loss(
            batch["params"], x_stacked, th.concatenate(control_inputs, axis=-1)
        )

        # (3) Apply loss that minimizes the lagrangian:
        # loss, values = self.energy_loss(batch["params"], x_out)

        with th.no_grad():
            self.training_log(
                batch=batch,
                values=values,
            )

        return loss

    def training_log(self, batch, values: T.Dict[str, th.Tensor]):
        if self.monitor is None:
            return
        self.monitor.log_values(values=values, step=self.global_step)


class DataModule(pl.LightningDataModule):
    """Module that generates input states for the system."""

    def __init__(
        self,
        version: str,
        num_examples: int,
        batch_size: int = 128,
        num_workers: int = 0,
    ):
        super().__init__()
        self.version = version
        self.num_examples = num_examples
        self.batch_size = batch_size
        self.num_workers = num_workers

        # Initially start states at zero w/ random angles.
        self.train_dataset = np.zeros(
            shape=(num_examples, 4 if self.version == "single" else 6), dtype=np.float32
        )
        self.train_dataset[:, 1] = np.random.uniform(
            low=np.pi / 2 - 0.2, high=np.pi / 2 + 0.2, size=len(self.train_dataset)
        )

        # Apply same pendulum params to every example for now.
        self.params = PendulumParams()
        self.params.m_b = 10.0
        self.params.m_1 = 0.25
        self.params.m_2 = 0.25
        self.params.l_1 = 0.15
        self.params.l_2 = 0.15
        self.params.g = 9.81

    def transform_collate(self, batch):
        batch_dicts = []
        for sample in batch:
            params_dict = th.tensor(
                [
                    self.params.m_b,
                    self.params.m_1,
                    self.params.m_2,
                    self.params.l_1,
                    self.params.l_2,
                    self.params.g,
                ],
                dtype=th.float32,
            )
            batch_dicts.append(dict(params=params_dict, x0_state=sample))
        return td.dataloader.default_collate(batch_dicts)

    def train_dataloader(self):
        return self.make_loader(self.train_dataset, shuffle=True)

    def val_dataloader(self):
        return None  # TODO: Implement.

    def make_loader(self, dataset, shuffle: bool) -> td.DataLoader:
        return td.DataLoader(
            dataset,
            batch_size=self.batch_size,
            num_workers=self.num_workers,
            shuffle=shuffle,
            collate_fn=self.transform_collate,
            drop_last=True,
        )


def main(args: argparse.Namespace):
    pl.seed_everything(7)
    data_module = DataModule(version=args.version, num_examples=200000)
    callbacks = [ModelCheckpoint(save_last=True)]
    monitor = Monitor(run_dir=SCRIPT_PATH / "logs")
    system = System(hparams=dict(**vars(args)), monitor=monitor)
    # system = System.load_from_checkpoint('model/logs/lightning_logs/version_0/checkpoints/last.ckpt')
    system.monitor = monitor

    trainer = pl.Trainer(max_epochs=10, logger=monitor, callbacks=callbacks)
    trainer.fit(model=system, train_dataloaders=data_module.train_dataloader())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--version",
        choices=["single", "double"],
        help="Which system to simulate",
        required=True,
    )
    return parser.parse_args()


if __name__ == "__main__":
    main(parse_args())
