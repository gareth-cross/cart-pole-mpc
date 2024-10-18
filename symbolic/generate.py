"""
Code-generate the forward dynamics of a double pendulum using wrenfold.
"""

import argparse
import subprocess
from pathlib import Path

from wrenfold import ast, code_generation, type_info

from .dynamics import get_double_pendulum_dynamics, get_single_pendulum_dynamics

REPO_ROOT = Path(__file__).parent.parent.absolute()


TYPESCRIPT_PREAMBLE = """
// Machine generated code (see generate.py)
import * as mathjs from 'mathjs';
import { PendulumParams } from './pendulum_params';

""".lstrip()


class CppCodeGenerator(code_generation.CppGenerator):
    """Place custom types into custom namespace."""

    def format_custom_type(self, custom: type_info.CustomType) -> str:
        return f"pendulum::{custom.name}"

    def format_matrix_type(self, mat: type_info.MatrixType) -> str:
        return f"Eigen::Matrix<Scalar, {mat.rows}, {mat.cols}>"

    def format_construct_matrix(self, construct: ast.ConstructMatrix) -> str:
        formatted_args = ", ".join(self.format(x) for x in construct.args)
        return f"Eigen::Matrix<double, {construct.type.rows}, {construct.type.cols}>({formatted_args})"


def main(args: argparse.Namespace):
    if args.version == "single":
        func = get_single_pendulum_dynamics()
    elif args.version == "double":
        func = get_double_pendulum_dynamics()

    code = code_generation.generate_function(func, generator=CppCodeGenerator())
    output_path = REPO_ROOT / "optimization" / f"{func.__name__}.hpp"
    with open(output_path, "w") as handle:
        handle.write(
            CppCodeGenerator.apply_preamble(
                code=code, namespace="gen", imports='#include "structs.hpp"'
            )
        )
        handle.flush()

    # Format it for good measure:
    subprocess.check_call(["clang-format", "-i", str(output_path)])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--version",
        choices=["single", "double"],
        help="Which system to generate",
        required=True,
    )
    return parser.parse_args()


if __name__ == "__main__":
    main(parse_args())
