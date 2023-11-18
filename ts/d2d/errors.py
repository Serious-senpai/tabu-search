from __future__ import annotations

from ..errors import TabuSearchException


__all__ = (
    "D2DSolverException",
    "ImportException",
    "NeighborhoodException",
)


class D2DSolverException(TabuSearchException):
    pass


class ImportException(D2DSolverException):

    def __init__(self, problem: str, /) -> None:
        super().__init__(f"Unable to import problem {problem!r}")


class NoProblemImported(D2DSolverException):

    def __init__(self) -> None:
        super().__init__("No problem was imported")


class NeighborhoodException(D2DSolverException):
    pass
