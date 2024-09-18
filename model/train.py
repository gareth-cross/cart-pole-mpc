"""
Train a model to control double pendulum.
"""

import argparse
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
            x_out, x_D_u = evaluate_forward_dynamics_single(
                params_structs, dt, u_controls.detach().cpu(), x0_states
            )
        else:
            x_out, x_D_u = evaluate_forward_dynamics_double(
                params_structs, dt, u_controls.detach().cpu(), x0_states
            )

        # Save the derivatives for the backward pass:
        ctx.save_for_backward(th.tensor(x_D_u, dtype=th.float32))
        return th.tensor(x_out, dtype=th.float32)

    @staticmethod
    def backward(ctx, grad_output: th.Tensor):
        # `u_controls` had shape (B, N).
        # `grad_output` will have shape (B, N, D), the same as `x_out`.
        # We need to output something with the same shape as `u_controls`: (B, N)
        (x_D_u,) = ctx.saved_tensors

        # x_D_u is ordered over: [batch, x_out, u_control, D]
        # We swap it such that the last two dims are: [..., x_out, D]
        x_D_u = th.permute(x_D_u, (0, 2, 1, 3))

        # Insert a new dimension and multiply loss_D_x with x_D_u
        grad_result = th.multiply(grad_output[:, None, :, :], x_D_u)

        # For each `u`, sum over all `xs` and complete the 1xD * Dx1 chain rule of:
        #   loss_D_x[i] * x[i]_D_u
        grad_result = th.sum(grad_result, dim=[-2, -1])

        # only `u_controls` will have derivatives:
        return None, None, grad_result, None


class EnergyLoss(nn.Module):
    """Evaluate loss based on Lagrangian of the pendulum."""

    def __init__(self, is_single: bool):
        super().__init__()
        self.is_single = is_single

    @staticmethod
    def evaluate_single_loss(params: th.Tensor, x_states: th.Tensor):
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
        if False:
            lin_spaced = th.linspace(0.0, 1.0, N, dtype=T.dtype)[None, ...]
            weighting = 3 * (lin_spaced**2) - 2 * (lin_spaced**3)
        else:
            weighting = th.zeros(size=(1, N), dtype=T.dtype)
            weighting[:, -1] = 1.0

        loss = th.mean((T - V) * weighting) / th.sum(weighting)
        return (loss, th.mean(T), th.mean(V))

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

    def forward(self, params: th.Tensor, x_states: th.Tensor):
        if self.is_single:
            return self.evaluate_single_loss(params=params, x_states=x_states)
        else:
            return self.evaluate_double_loss(params=params, x_states=x_states)


class Network(nn.Module):
    """Network to control the pendulum."""

    def __init__(self, input_dim: int) -> None:
        super().__init__()
        self.something = nn.Sequential(
            nn.Linear(input_dim, 64), nn.Tanh(), nn.Linear(64, 200)
        )

    def forward(self, x_initial_states: th.Tensor) -> th.Tensor:
        return self.something(x_initial_states)


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

    def __init__(
        self, hparams: argparse.Namespace, monitor: T.Optional[Monitor] = None
    ):
        super().__init__()
        self.is_single = hparams.version == "single"
        self.model = Network(input_dim=4 if self.is_single else 6)
        self.monitor = monitor
        self.energy_loss = EnergyLoss(is_single=self.is_single)
        self.save_hyperparameters(hparams)

    def configure_optimizers(self):
        return th.optim.Adam(self.parameters(), lr=0.001)

    def training_step(self, batch):
        # (1) Query network for trajectory:
        u_output = self.model(batch["x0_state"])

        # (2) Integrate the dynamics model:
        x_out = DynamicsLayer.apply(batch["params"], 0.01, u_output, batch["x0_state"])

        # (3) Apply loss that minimizes the lagrangian:
        loss, kinetic, potential = self.energy_loss(batch["params"], x_out)

        with th.no_grad():
            self.training_log(
                batch=batch,
                values=dict(loss=loss, kinetic=kinetic, potential=potential),
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
        batch_size: int = 16,
        num_workers: int = 0,
    ):
        super().__init__()
        self.version = version
        self.num_examples = num_examples
        self.batch_size = batch_size
        self.num_workers = num_workers
        # Initially start states at zero:
        self.train_dataset = np.zeros(
            shape=(num_examples, 4 if self.version == "single" else 6), dtype=np.float32
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
    data_module = DataModule(version=args.version, num_examples=100000)
    callbacks = [ModelCheckpoint(save_last=True)]
    monitor = Monitor(run_dir=SCRIPT_PATH / "logs")
    system = System(hparams=args, monitor=monitor)
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
