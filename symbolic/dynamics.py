import dataclasses
import typing as T

from wrenfold import code_generation, sym, type_annotations

from .sympy_utils import get_euler_lagrange_coefficients, get_mat_inverse


@dataclasses.dataclass
class SingleCartPoleParams:
    """Parameters of the single-pole system."""

    m_b: type_annotations.FloatScalar
    m_1: type_annotations.FloatScalar
    l_1: type_annotations.FloatScalar
    g: type_annotations.FloatScalar
    mu_b: type_annotations.FloatScalar
    v_mu_b: type_annotations.FloatScalar


@dataclasses.dataclass
class DoubleCartPoleParams:
    """Parameters of the double-pole system."""

    m_b: type_annotations.FloatScalar
    m_1: type_annotations.FloatScalar
    m_2: type_annotations.FloatScalar
    l_1: type_annotations.FloatScalar
    l_2: type_annotations.FloatScalar
    g: type_annotations.FloatScalar


def get_double_pendulum_dynamics() -> T.Callable:
    """
    Return a symbolic function that evaluates the dynamics of a cart-mounted double pendulum.
    """
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

    M_inv, m_symbols = get_mat_inverse(dim=3)
    substitutions = list(zip(m_symbols, A.to_flat_list()))

    # Compute expressions for x'' = A(x, x')^-1 * f(x, x', u)
    A_inv = M_inv.subs(substitutions)
    x_ddot = A_inv * f

    def double_pendulum_dynamics(
        params: DoubleCartPoleParams,
        x: type_annotations.Vector6,
        u: type_annotations.FloatScalar,
    ):
        """
        Evaluates the forward dynamics.
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

    return double_pendulum_dynamics


def get_single_pendulum_dynamics() -> T.Callable:
    """
    Return a symbolic function that evaluates the dynamics of a cart-mounted single pendulum.

    There is a bit of copy-pasta in writing this function (taken from the one above).
    """
    t = sym.symbols("t", real=True)

    # Position of the base + pole angle as a function of time.
    b_x = sym.Function("b_x")(t)
    th_1 = sym.Function("th_1")(t)

    # First derivatives of the base + angles:
    b_x_dot = b_x.diff(t)
    th_1_dot = th_1.diff(t)

    # Mass of the base, and two weights + lever arm lengths:
    m_b, m_1, l_1 = sym.symbols("m_b, m_1, l_1", real=True, positive=True)

    # Gravity:
    g = sym.symbols("g", real=True)

    # Control input on the base:
    u_b = sym.symbols("u_b", real=True)

    # Positions of base, and pole mounted weight.
    b = sym.vector(b_x, 0)
    p_1 = b + sym.vector(sym.cos(th_1), sym.sin(th_1)) * l_1
    b_dot = b.diff(t)
    p_1_dot = p_1.diff(t)

    # Friction coefficient on the base, and the cutoff velocity of the smooth Coulomb model.
    mu_b = sym.symbols("mu_b", real=True)
    v_mu_b = sym.symbols("v_mu_b", real=True)

    # Compute kinetic energy. This is the sum of (1/2)*m*v^2 for all pieces.
    half = 1 / sym.integer(2)
    T: sym.Expr = (
        half * m_b * b_dot.squared_norm() + half * m_1 * p_1_dot.squared_norm()
    )

    # Simplify this a bit by eliminating: cos^2(x) + sin^2(x) --> 1
    T = (
        T.distribute()
        .collect([m_1, l_1, th_1_dot])
        .subs((sym.cos(th_1) ** 2 + sym.sin(th_1) ** 2) / 2, half)
    )

    # Compute potential energy. This is m*g*y for the pole-mounted mass.
    V = g * m_1 * p_1[1]

    # The lagrangian:
    L: sym.Expr = T - V

    # Compute the canonical momenta.
    # We need a temporary variable to take derivative wrt, because wrenfold cannot yet take the
    # derivative wrt the Derivative(b(x), t) expression. I will hopefully fix this limitation soon.
    alpha = sym.symbols("alpha", real=True)
    q_b = L.subs(b_x_dot, alpha).diff(alpha).subs(alpha, b_x_dot)
    q_th_1 = L.subs(th_1_dot, alpha).diff(alpha).subs(alpha, th_1_dot)

    # External forces applied at the base and at the point mass:
    f_b = sym.vector(*sym.symbols("f_b_x, f_b_y"))
    f_m_1 = sym.vector(*sym.symbols("f_m1_x, f_m1_y"))

    # Compute generalized forces:
    (Q_b,) = f_b.T * b.diff(b_x) + f_m_1.T * p_1.diff(b_x)
    (Q_th,) = f_b.T * b.diff(th_1) + f_m_1.T * p_1.diff(th_1)

    # Dissipative force due to friction on the base.
    F_d_base = mu_b * (m_1 + m_b) * g * sym.tanh(b_x_dot / v_mu_b)

    # Form the Euler-Lagrange equations (each of these is equal to zero).
    # We add in our control input `u_b`, which is a non-conservative force.
    el_b = (q_b.diff(t) - L.diff(b_x)).distribute() - u_b - Q_b + F_d_base
    el_th_1 = (q_th_1.diff(t) - L.diff(th_1)).distribute() - Q_th

    # Reformulate the Euler-Lagrange equations into form:
    #   A(x, x') * x'' = f(x, x', u)
    A, f = get_euler_lagrange_coefficients(
        euler_lagrange=[el_b, el_th_1],
        second_derivatives=[b_x.diff(t, 2), th_1.diff(t, 2)],
    )

    M_inv, m_symbols = get_mat_inverse(dim=2)
    substitutions = list(zip(m_symbols, A.to_flat_list()))

    # Compute expressions for x'' = A(x, x')^-1 * f(x, x', u)
    A_inv = M_inv.subs(substitutions).collect(l_1)
    x_ddot = A_inv * f

    def single_pendulum_dynamics(
        params: SingleCartPoleParams,
        x: type_annotations.Vector4,
        u: type_annotations.FloatScalar,
        f_base: type_annotations.Vector2,
        f_mass: type_annotations.Vector2,
    ):
        """
        Evaluates the forward dynamics.
        """
        states = list(zip([b_x, th_1], x[:2].to_flat_list()))
        vel_states = list(zip([b_x_dot, th_1_dot], x[2:].to_flat_list()))
        x_ddot_subbed = (
            x_ddot.subs(
                [
                    (m_b, params.m_b),
                    (m_1, params.m_1),
                    (l_1, params.l_1),
                    (g, params.g),
                    (mu_b, params.mu_b),
                    (v_mu_b, params.v_mu_b),
                ]
            )
            .subs(vel_states)
            .subs([(u_b, u)] + states)
            .subs(
                [
                    (f_b[0], f_base[0]),
                    (f_b[1], f_base[1]),
                    (f_m_1[0], f_mass[0]),
                    (f_m_1[1], f_mass[1]),
                ]
            )
        )

        # Stack the first derivative with the second derivative.
        # This is how we get the 4-element derivative of our state vector.
        x_dot_out = sym.vstack([x[2:], x_ddot_subbed])

        # Compute Jacobians of the state derivative wrt the state and the control input.
        J_x = sym.jacobian(x_dot_out, x)
        J_u = sym.jacobian(x_dot_out, [u])

        return [
            code_generation.OutputArg(x_dot_out, name="x_dot"),
            code_generation.OutputArg(J_x, name="J_x", is_optional=True),
            code_generation.OutputArg(J_u, name="J_u", is_optional=True),
        ]

    return single_pendulum_dynamics
