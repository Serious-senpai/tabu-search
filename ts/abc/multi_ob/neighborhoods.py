from __future__ import annotations

from multiprocessing import pool
from typing import Iterable, TypeVar

from .solutions import MultiObjectiveSolution
from ..types import BaseNeighborhood


__all__ = ("MultiObjectiveNeighborhood",)
_MultiST = TypeVar("_MultiST", bound=MultiObjectiveSolution)
_TT = TypeVar("_TT")


class MultiObjectiveNeighborhood(BaseNeighborhood[_MultiST, _TT]):
    """Base class for neighborhoods of a solution to a multi-objective optimization problem"""

    def find_best_candidates(self, *, pool: pool.Pool, pool_size: int) -> Iterable[_MultiST]:
        """Find all non-dominant solutions in this neighborhood of the current solution.

        Subclasses must implement this.

        Parameters
        -----
        pool: `pool.Pool`
            The process pool to perform the operation
        pool_size: `int`
            The process pool size

        Returns
        -----
        Iterable[`MultiObjectiveSolution`]
            The set of non-dominant solutions
        """
        raise NotImplementedError
