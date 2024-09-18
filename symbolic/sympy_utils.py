"""
Some utilities that use SymPy to help with the symbolic derivation.
"""

import typing as T

import sympy as sp
from wrenfold import sym, sympy_conversion


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


def get_mat_inverse(dim: int) -> T.Tuple[sym.MatrixExpr, T.List[sym.Expr]]:
    """
    Use SymPy to get closed formed inverse for a matrix. This really only works for 2x2 and 3x3.
    """
    syms = sp.symbols(f"c0:{dim}_0:{dim}")
    elements = sp.Matrix(syms).reshape(rows=dim, cols=dim)
    M_inv = sympy_conversion.from_sympy(elements.inv())
    return M_inv, [sympy_conversion.from_sympy(s) for s in syms]
