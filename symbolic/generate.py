"""
Code-generate the forward dynamics of a double pendulum using wrenfold.
"""

import argparse
import subprocess
from pathlib import Path

from wrenfold import code_generation, type_info

from .dynamics import get_double_pendulum_dynamics, get_single_pendulum_dynamics
from .ts_generator import TypeScriptCodeGenerator

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


def main(args: argparse.Namespace):
    if args.version == "single":
        functions = get_single_pendulum_dynamics()
    elif args.version == "double":
        functions = get_double_pendulum_dynamics()

    for func in functions:
        if args.target == "ts":
            code = code_generation.generate_function(
                func, generator=TypeScriptCodeGenerator()
            )
            with open(
                REPO_ROOT / "site" / "src" / f"{func.__name__}.ts", "w"
            ) as handle:
                handle.write(TYPESCRIPT_PREAMBLE + code)
        elif args.target == "cpp":
            code = code_generation.generate_function(func, generator=CppCodeGenerator())
            output_path = REPO_ROOT / "optimization" / f"{func.__name__}.hpp"
            with open(output_path, "w") as handle:
                handle.write(
                    CppCodeGenerator.apply_preamble(
                        code=code, namespace="gen", imports='#include "parameters.hpp"'
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
    parser.add_argument(
        "--target",
        choices=["ts", "cpp"],
        help="Which implementation to generate",
        required=True,
    )
    return parser.parse_args()


if __name__ == "__main__":
    main(parse_args())
