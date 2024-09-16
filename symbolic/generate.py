import dataclasses
import typing as T
import sympy as sp
from pathlib import Path

from wrenfold import code_generation, sym, sympy_conversion, type_annotations

from .ts_generator import TypeScriptCodeGenerator

REPO_ROOT = Path(__file__).parent.parent.absolute()


def get_euler_lagrange_coefficients(
    euler_lagrange: T.List[sym.Expr], second_derivatives: T.List[sym.Expr]
) -> T.Tuple[sym.MatrixExpr, sym.MatrixExpr]:
    """
    Given the Euler-Lagrange equations, extract coefficients on `second_derivatives` so that
    we can write the system in the form:

    A(x, x') * x'' = f(x, x', u)
    """
    A_rows = []
    b_rows = []
    for eq in euler_lagrange:
        eq_sp = sympy_conversion.to_sympy(eq)
        row = []
        for sd in second_derivatives:
            sd_sp = sympy_conversion.to_sympy(sd)
            coeff = eq_sp.coeff(sd_sp)
            row.append(sympy_conversion.from_sympy(coeff))

            # Remove term on this derivative.
            eq_sp = eq_sp - (coeff * sd_sp).expand()

        A_rows.append(row)

        # Negate eq_sp as we move it to the RHS of the system of equations.
        b_rows.append(sympy_conversion.from_sympy(-eq_sp))

    return sym.matrix(A_rows), sym.matrix(b_rows)


def get_mat_inverse() -> T.Tuple[sym.MatrixExpr, T.List[sym.Expr]]:
    """Use SymPy to get closed formed inverse for a 3x3 matrix."""
    syms = sp.symbols("c0:3_0:3")
    elements = sp.Matrix(syms).reshape(rows=3, cols=3)
    M_inv = sympy_conversion.from_sympy(elements.inv())
    return M_inv, [sympy_conversion.from_sympy(s) for s in syms]


@dataclasses.dataclass
class PendulumParams:
    """Parameters of the pendulum system."""

    m_b: type_annotations.FloatScalar
    m_1: type_annotations.FloatScalar
    m_2: type_annotations.FloatScalar
    l_1: type_annotations.FloatScalar
    l_2: type_annotations.FloatScalar
    g: type_annotations.FloatScalar


def main():
    t = sym.symbols("t", real=True)

    # Position of the base + angles as a function of time:
    b_x = sym.Function("b_x")(t)
    th_1 = sym.Function("th_1")(t)
    th_2 = sym.Function("th_2")(t)

    # First derivatives of the base + angles:
    b_x_dot = b_x.diff(t)
    th_1_dot = th_1.diff(t)
    th_2_dot = th_2.diff(t)

    # Mass of the base, and two weights + lever arm lengths:
    m_b, m_1, m_2, l_1, l_2 = sym.symbols(
        "m_b, m_1, m_2, l_1, l_2", real=True, positive=True
    )

    # Gravity:
    g = sym.symbols("g", real=True)

    # Control input on the base:
    u_b = sym.symbols("u_b", real=True)

    # Positions of base, and two weights.
    b = sym.vector(b_x, 0)
    p_1 = b + sym.vector(sym.cos(th_1), sym.sin(th_1)) * l_1
    p_2 = p_1 + sym.vector(sym.cos(th_2), sym.sin(th_2)) * l_2

    b_dot = b.diff(t)
    p_1_dot = p_1.diff(t)
    p_2_dot = p_2.diff(t)

    # Compute kinetic energy. This is the sum of (1/2)*m*v^2 for all pieces.
    half = 1 / sym.integer(2)
    T: sym.Expr = (
        half * m_b * b_dot.squared_norm()
        + half * m_1 * p_1_dot.squared_norm()
        + half * m_2 * p_2_dot.squared_norm()
    )

    # Simplify this a bit by eliminating: cos^2(x) + sin^2(x) --> 1
    T = (
        T.distribute()
        .collect([m_1, m_2, l_1, l_2, th_1_dot, th_2_dot])
        .subs((sym.cos(th_1) ** 2 + sym.sin(th_1) ** 2) / 2, half)
        .subs((sym.cos(th_2) ** 2 + sym.sin(th_2) ** 2) / 2, half)
    )

    # Compute potential energy. This is the sum of m*g*y for all pieces.
    V = g * m_1 * p_1[1] + g * m_2 * p_2[1]

    # The lagrangian:
    L: sym.Expr = T - V

    # Canonical momenta - we need a temporary variable to take derivative wrt.
    alpha = sym.symbols("alpha", real=True)
    q_b = L.subs(b_x_dot, alpha).diff(alpha).subs(alpha, b_x_dot)
    q_th_1 = L.subs(th_1_dot, alpha).diff(alpha).subs(alpha, th_1_dot)
    q_th_2 = L.subs(th_2_dot, alpha).diff(alpha).subs(alpha, th_2_dot)

    # Form the Euler-Lagrange equations (each of these is equal to zero).
    # We add in our control input `u_b`, which is a non-conservative force.
    el_b = (q_b.diff(t) - L.diff(b_x)).distribute() - u_b
    el_th_1 = (q_th_1.diff(t) - L.diff(th_1)).distribute()
    el_th_2 = (q_th_2.diff(t) - L.diff(th_2)).distribute()

    # Reformulate the Euler-Lagrange equations into form:
    #   A(x, x') * x'' = f(x, x', u)
    A, f = get_euler_lagrange_coefficients(
        euler_lagrange=[el_b, el_th_1, el_th_2],
        second_derivatives=[b_x.diff(t, 2), th_1.diff(t, 2), th_2.diff(t, 2)],
    )

    M_inv, m_symbols = get_mat_inverse()
    substitutions = list(zip(m_symbols, A.to_flat_list()))

    # Compute expressions for x'' = A(x, x')^-1 * f(x, x', u)
    A_inv = M_inv.subs(substitutions)
    x_ddot = A_inv * f

    def pendulum_dynamics(
        params: PendulumParams,
        x: type_annotations.Vector6,
        u: type_annotations.FloatScalar,
    ):
        """
        Evaluates the pendulum dynamics. Outputs a
        """
        states = list(zip([b_x, th_1, th_2], x[:3].to_flat_list()))
        vel_states = list(zip([b_x_dot, th_1_dot, th_2_dot], x[3:].to_flat_list()))
        x_ddot_subbed = (
            x_ddot.subs(
                [
                    (m_b, params.m_b),
                    (m_1, params.m_1),
                    (m_2, params.m_2),
                    (l_1, params.l_1),
                    (l_2, params.l_2),
                    (g, params.g),
                ]
            )
            .subs(vel_states)
            .subs([(u_b, u)] + states)
        )

        # Stack the first derivative with the second derivative.
        # This is how we get the 6-element derivative of our state vector.
        x_dot_out = sym.vstack([x[3:], x_ddot_subbed])

        # Compute Jacobians of the state derivative wrt the state and the control input.
        J_x = sym.jacobian(x_dot_out, x)
        J_u = sym.jacobian(x_dot_out, [u])

        return [
            code_generation.OutputArg(x_dot_out, name="x_dot"),
            code_generation.OutputArg(J_x, name="J_x", is_optional=True),
            code_generation.OutputArg(J_u, name="J_u", is_optional=True),
        ]

    code = code_generation.generate_function(
        pendulum_dynamics, generator=TypeScriptCodeGenerator()
    )

    with open(REPO_ROOT / "site" / "src" / "dynamics.ts", "w") as handle:
        ts_preamble = '\n'.join([
            "import * as mathjs from 'mathjs';",
            "import { PendulumParams } from './pendulum_params';"
        ]) + "\n\n"
        handle.write(ts_preamble + code)


if __name__ == "__main__":
    main()
