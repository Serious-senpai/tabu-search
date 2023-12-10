from __future__ import annotations

from multiprocessing import pool
from typing import Optional, TypeVar

from .solutions import SingleObjectiveSolution
from ..bases import BaseNeighborhood


__all__ = ("SingleObjectiveNeighborhood",)
_SingleST = TypeVar("_SingleST", bound=SingleObjectiveSolution)
_TT = TypeVar("_TT")


class SingleObjectiveNeighborhood(BaseNeighborhood[_SingleST, _TT]):
    """Base class for neighborhoods of a solution to a single-objective optimization problem"""

    __slots__ = ()

    def find_best_candidate(self, *, pool: pool.Pool, pool_size: int) -> Optional[_SingleST]:
        """Find the best candidate solution within the neighborhood of the current one.

        Parameters
        -----
        pool:
            The process pool to perform the operation
        pool_size:
            The process pool size

        Returns
        -----
        The best candidate solution, or `None` if there are no candidates
        """
        raise NotImplementedError
