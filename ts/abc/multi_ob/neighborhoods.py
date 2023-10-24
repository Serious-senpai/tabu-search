from __future__ import annotations

from multiprocessing import pool
from typing import Set, TypeVar

from .solutions import MultiObjectiveSolution
from ..types import _BaseNeighborhood


__all__ = ("MultiObjectiveNeighborhood",)
_MultiST = TypeVar("_MultiST", bound=MultiObjectiveSolution)
_TT = TypeVar("_TT")


class MultiObjectiveNeighborhood(_BaseNeighborhood[_MultiST, _TT]):
    """Base class for neighborhoods of a solution to a multi-objective optimization problem"""

    def find_best_candidates(self, *, pool: pool.Pool, pool_size: int) -> Set[_MultiST]:
        """Find all non-dominant solutions in this neighborhood of the current solution.

        Parameters
        -----
        pool: `pool.Pool`
            The process pool to perform the operation
        pool_size: `int`
            The process pool size

        Returns
        -----
        Set[`MultiObjectiveSolution`]
            The set of non-dominant solutions
        """
        raise NotImplementedError
