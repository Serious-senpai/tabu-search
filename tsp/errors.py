from __future__ import annotations

from typing import Any, TYPE_CHECKING


__all__ = (
    "SolverException",
    "ProblemNotFound",
    "ProblemParsingException",
    "UnsupportedEdgeWeightType",
    "OptimalSolutionNotFound",
)


class SolverException(Exception):
    """Base class for all exceptions from this library"""
    pass


class ProblemNotFound(SolverException):
    """Exception raised when the problem is not found within the archive"""

    def __init__(self, problem: Any, /) -> None:
        super().__init__(f"Cannot find any problems with the given name: {problem!r}")


class ProblemParsingException(SolverException):
    """Exception raised when parsing the problem input fails"""

    __slots__ = (
        "original",
    )
    if TYPE_CHECKING:
        original: BaseException

    def __init__(self, problem: Any, original: BaseException, /) -> None:
        super().__init__(f"Cannot parse input for problem {problem!r}")
        self.original = original


class UnsupportedEdgeWeightType(SolverException):

    __slots__ = (
        "edge_weight_type",
    )
    if TYPE_CHECKING:
        edge_weight_type: str

    def __init__(self, edge_weight_type: str, /) -> None:
        self.edge_weight_type = edge_weight_type
        super().__init__(f"Unsupported edge_weight_type {edge_weight_type!r}")


class OptimalSolutionNotFound(SolverException):
    """Exception raised when the *.opt.tour file not found for the current problem"""

    def __init__(self, problem: Any, /) -> None:
        super().__init__(f"Cannot find the optimal solution file for the given problem: {problem!r}")
