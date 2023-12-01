from __future__ import annotations

from ..errors import TabuSearchException


__all__ = (
    "D2DSolverException",
    "ProblemImportException",
    "NeighborhoodException",
)


class D2DSolverException(TabuSearchException):
    pass


class ProblemImportException(D2DSolverException):

    def __init__(self, problem: str, /) -> None:
        super().__init__(f"Unable to import problem {problem!r}")


class NeighborhoodException(D2DSolverException):
    pass
