"""
TypeScript code generator for wrenfold.
"""

import typing as T

import numpy as np
from wrenfold import ast, code_generation, geometry, sym, type_annotations, type_info
from wrenfold.enumerations import RelationalOperation, StdMathFunction, SymbolicConstant


class TypeScriptCodeGenerator(code_generation.BaseGenerator):
    """
    A custom code-generator that emits TypeScript.
    """

    def __init__(self, indent: int = 2):
        super().__init__()
        assert indent > 0, f"indent = {indent}"
        self._indent: str = " " * indent

    def _indent_and_join(self, lines: T.Iterable[str]) -> str:
        lines_split = []
        for line in lines:
            lines_split.extend(line.splitlines())
        return self._indent + f"\n{self._indent}".join(lines_split)

    @staticmethod
    def _format_numeric_type(t: type_info.NumericType) -> str:
        if t == type_info.NumericType.Bool:
            return "boolean"
        elif t == type_info.NumericType.Integer:
            return "number"
        elif t == type_info.NumericType.Float:
            return "number"
        else:
            raise NotImplementedError(f"Unsupported type: {t}")

    def format_scalar_type(self, t: type_info.ScalarType) -> str:
        return self._format_numeric_type(t.numeric_type)

    def format_matrix_type(self, _: type_info.MatrixType) -> str:
        return "mathjs.Matrix"

    def format_custom_type(self, custom: type_info.CustomType) -> str:
        return custom.name

    def format_add(self, add: ast.Add) -> str:
        return " + ".join(self.format(x) for x in add.args)

    def format_assign_output_matrix(self, mat: ast.AssignOutputMatrix) -> str:
        mat_type = mat.arg.type
        assert isinstance(mat_type, type_info.MatrixType)

        result = [f"{mat.arg.name}.resize([{mat_type.rows}, {mat_type.cols}])"]
        for idx, arg in enumerate(mat.value.args):
            row, col = mat_type.compute_indices(idx)
            result.append(f"{mat.arg.name}.set([{row}, {col}], {self.format(arg)})")

        return "\n".join(result)

    def format_assign_output_scalar(self, scalar: ast.AssignOutputScalar) -> str:
        raise NotImplementedError("Cannot assign to output scalars in TypeScript")

    def format_assign_output_struct(self, struct: ast.AssignOutputStruct) -> str:
        # TODO: We could implement this by assigning to object members.
        raise NotImplementedError("Cannot assign to output struct in TypeScript")

    def format_assign_temporary(self, temp: ast.AssignTemporary) -> str:
        return f"{temp.left} = {self.format(temp.right)}"

    def format_boolean_literal(self, b: ast.BooleanLiteral) -> str:
        return "true" if b.value else "false"

    def format_branch(self, branch: ast.Branch) -> str:
        result = f"if ({self.format(branch.condition)}) {{\n"
        result += self._indent_and_join(self.format(x) for x in branch.if_branch)
        if len(branch.else_branch) > 0:
            result += "\n} else:\n"
            result += self._indent_and_join(self.format(x) for x in branch.else_branch)
            result += "\n}"
        return result

    def format_call_external_function(self, call: ast.CallExternalFunction) -> str:
        return (
            f"{call.function.name}("
            + ", ".join(self.format(x) for x in call.args)
            + ")"
        )

    def format_call_std_function(self, call: ast.CallStdFunction) -> str:
        functions = {
            StdMathFunction.Cos: "cos",
            StdMathFunction.Sin: "sin",
            StdMathFunction.Tan: "tan",
            StdMathFunction.Acos: "acos",
            StdMathFunction.Asin: "asin",
            StdMathFunction.Atan: "atan",
            StdMathFunction.Sqrt: "sqrt",
            StdMathFunction.Cosh: "cosh",
            StdMathFunction.Sinh: "sinh",
            StdMathFunction.Tanh: "tanh",
            StdMathFunction.Acosh: "acosh",
            StdMathFunction.Asinh: "asinh",
            StdMathFunction.Atanh: "atanh",
            StdMathFunction.Abs: "abs",
            StdMathFunction.Log: "log",
            StdMathFunction.Signum: "sign",
            StdMathFunction.Floor: "floor",
            StdMathFunction.Atan2: "atan2",
            StdMathFunction.Powi: "pow",
            StdMathFunction.Powf: "pow",
        }
        return (
            f"mathjs.{functions[call.function]}("
            + ", ".join(self.format(x) for x in call.args)
            + ")"
        )

    def format_cast(self, cast: ast.Cast) -> str:
        arg = self.format(cast.arg)
        if cast.destination_type == type_info.NumericType.Bool:
            return f"Boolean({arg})"
        elif cast.destination_type == type_info.NumericType.Integer:
            return f"Number(Math.trunc({arg}))"
        elif cast.destination_type == type_info.NumericType.Float:
            return f"Number({arg})"
        else:
            raise NotImplementedError(f"Unsupported type: {cast.destination_type}")

    def format_comment(self, comment: ast.Comment) -> str:
        return "\n".join(f"// {x}" for x in comment.split_lines())

    def format_compare(self, compare: ast.Compare) -> str:
        if compare.operation == RelationalOperation.LessThan:
            op = "<"
        elif compare.operation == RelationalOperation.LessThanOrEqual:
            op = "<="
        elif compare.operation == RelationalOperation.Equal:
            op = "==="
        else:
            raise NotImplementedError(f"Unknown operation: {compare.operation}")
        return f"{self.format(compare.left)} {op} {self.format(compare.right)}"

    def format_construct_matrix(self, matrix: ast.ConstructMatrix) -> str:
        mat_type = matrix.value.type
        array = np.empty(mat_type.shape, dtype=str)
        for i, arg in matrix.value.args:
            row, col = mat_type.compute_indices(i)
            array[row, col] = self.format(arg)

        rows = [f"[{', '.join(row)}]" for row in array]
        return f'mathjs.matrix([{", ".join(rows)}])'

    def format_construct_custom_type(self, custom: ast.ConstructCustomType) -> str:
        fields = ", ".join(
            f"{self.format(custom.get_field_value(f.name))}" for f in custom.type.fields
        )
        return f"{self.format(custom.type)}({fields})"

    def format_declaration(self, decl: ast.Declaration) -> str:
        if decl.value is not None:
            return f"const {decl.name} = {self.format(decl.value)};"
        else:
            return f"let {decl.name};"

    def format_divide(self, div: ast.Divide) -> str:
        return f"{self.format(div.left)} / {self.format(div.right)}"

    def format_float_literal(self, flt: ast.FloatLiteral) -> str:
        return str(flt.value)

    def format_get_argument(self, get: ast.GetArgument) -> str:
        return get.argument.name

    def format_get_field(self, get: ast.GetField) -> str:
        return f"{self.format(get.arg)}.{get.field_name}"

    def format_get_matrix_element(self, get: ast.GetMatrixElement) -> str:
        return f"{self.format(get.arg)}.get([{get.row}, {get.col}])"

    def format_integer_literal(self, i: ast.IntegerLiteral) -> str:
        return str(i.value)

    def format_multiply(self, mul: ast.Multiply) -> str:
        return " * ".join(self.format(x) for x in mul.args)

    def format_negate(self, neg: ast.Negate) -> str:
        return f"-{self.format(neg.arg)}"

    def format_optional_output_branch(self, branch: ast.OptionalOutputBranch) -> str:
        result = f"if (compute_{branch.argument.name}) {{\n"
        result += self._indent_and_join(self.format(x) for x in branch.statements)
        result += "\n}"
        return result

    def format_parenthetical(self, parenthetical: ast.Parenthetical) -> str:
        return f"({self.format(parenthetical.contents)})"

    def format_return_object(self, ret: ast.ReturnObject) -> str:
        raise NotImplementedError("Handled in function body")

    def format_special_constant(self, constant: ast.SpecialConstant) -> str:
        if constant.value == SymbolicConstant.Euler:
            return "mathjs.E"
        elif constant.value == SymbolicConstant.Pi:
            return "mathjs.PI"
        else:
            raise NotImplementedError(f"Unsupported constant: {constant.value}")

    def format_variable_ref(self, var: ast.VariableRef) -> str:
        return var.name

    def format_function_definition(self, definition: ast.FunctionDefinition) -> str:
        """
        The top-level formatting function. The `FunctionDefinition` object is the root of the
        syntax tree.

        Args:
            definition: ast.FunctionDefinition
        """
        # Insert some reshape calls to make sure matrix inputs are accessible with 2D slicing.
        lines: T.List[str] = []
        lines.append("// Check input dimensions:")
        for arg in definition.signature.arguments:
            if arg.direction == code_generation.ArgumentDirection.Input and isinstance(
                arg.type, type_info.MatrixType
            ):
                lines.extend(
                    [
                        f"if ({arg.name}.size()[0] !== {arg.type.rows} || {arg.name}.size()[1] !== {arg.type.cols}) {{",
                        f"{self._indent}throw new Error(`Wrong shape for {arg.name}: ${{{arg.name}.size()}}`)",
                        "}",
                    ]
                )

        # Declare outputs:
        lines.append("// Declare output arrays:")
        for arg in definition.signature.arguments:
            if arg.direction != code_generation.ArgumentDirection.Input and isinstance(
                arg.type, type_info.MatrixType
            ):
                lines.append(f"const {arg.name} = mathjs.matrix([]);")

        # Generate every statement in the body, but omit the return statement.
        # We customize the return below to include output args.
        lines.extend(
            [
                self.format(x)
                for x in definition.body
                if not isinstance(x, ast.ReturnObject)
            ]
        )

        # Figure out what we need to include in the returned object:
        returned: T.List[T.Tuple[str, str]] = []
        if isinstance(definition.body[-1], ast.ReturnObject):
            return_value = T.cast(ast.ReturnObject, definition.body[-1]).value
            # TODO: Ensure this name is unique.
            returned.append((return_value, self.format(return_value)))

        for arg in definition.signature.arguments:
            if arg.direction != code_generation.ArgumentDirection.Input:
                returned.append((arg.name, arg.name))

        members = [f"{k}: {v}" for (k, v) in returned]
        lines.append("return { " + ", ".join(members) + " }")
        return (
            self.format(definition.signature)
            + " {\n"
            + self._indent_and_join(lines)
            + "\n}"
        )

    def format_function_signature(self, sig: ast.FunctionSignature) -> str:
        args = []
        outputs: T.List[str] = []
        for arg in sig.arguments:
            if arg.direction == code_generation.ArgumentDirection.Input:
                args.append(f"{arg.name}: {self.format(arg.type)}")
            else:
                outputs.append(f"{arg.name}: {self.format(arg.type)}")
                if arg.is_optional:
                    # For output args, create a bool to indicate if it should be computed
                    args.append(f"compute_{arg.name}: boolean")

        if sig.return_type is not None:
            outputs.append(f"return_value: {self.format(sig.return_type)}")

        if outputs:
            return_annotation = f": {{ {', '.join(outputs)} }}"
        else:
            return_annotation = ""

        return f'export function {sig.name}({", ".join(args)}){return_annotation}'
