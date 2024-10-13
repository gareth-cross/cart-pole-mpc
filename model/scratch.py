"""
Visualize some things.
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).parent.parent.absolute()
sys.path.insert(0, str(REPO_ROOT / "build" / "wrapper"))

from pypendulum import (
    Optimization,
    OptimizationParams,
    SingleCartPoleParams,
    SingleCartPoleState,
)


def main():
    params = SingleCartPoleParams(1.0, 0.1, 0.25, 9.81)
    x0_initial = SingleCartPoleState(0.0, 0.0, 0.0, 0.0)

    opt_params = OptimizationParams()
    opt = Optimization(opt_params)
    outputs = opt.step(x0_initial, params)

    fig, ((ax0, ax1), (ax2, ax3), (ax4, _)) = plt.subplots(nrows=3, ncols=2)

    # angle and angle derivative
    ax0.plot([s.th_1 for s in outputs.predicted_states])
    ax0.set_xlabel("Time")
    ax0.set_ylabel("Radians")
    ax0.grid()

    ax1.plot([s.th_1_dot for s in outputs.predicted_states])
    ax1.set_xlabel("Time")
    ax1.set_ylabel("Radians / s")
    ax1.grid()

    # position and position derivative
    ax2.plot([s.b_x for s in outputs.predicted_states])
    ax2.set_xlabel("Time")
    ax2.set_ylabel("Meters")
    ax2.grid()

    ax3.plot([s.b_x_theta for s in outputs.predicted_states])
    ax3.set_xlabel("Time")
    ax3.set_ylabel("Meters / s")
    ax3.grid()

    ax4.plot(outputs.u)
    ax4.set_xlabel("Time")
    ax4.set_ylabel("Newtons")
    ax4.grid()

    plt.show()


if __name__ == "__main__":
    main()
