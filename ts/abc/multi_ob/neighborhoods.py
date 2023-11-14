from __future__ import annotations

from multiprocessing import pool
from typing import Any, Callable, Iterable, Optional, TypeVar

from .solutions import MultiObjectiveSolution
from ..bases import BaseNeighborhood


__all__ = ("MultiObjectiveNeighborhood",)
_MultiST = TypeVar("_MultiST", bound=MultiObjectiveSolution)
_TT = TypeVar("_TT")


class MultiObjectiveNeighborhood(BaseNeighborhood[_MultiST, _TT]):
    """Base class for neighborhoods of a solution to a multi-objective optimization problem"""

    __slots__ = ()

    def find_best_candidates(self, *, pool: pool.Pool, pool_size: int, logger: Optional[Callable[[str], Any]]) -> Iterable[_MultiST]:
        """Find all non-dominant solutions in this neighborhood of the current solution.

        Subclasses must implement this.

        Parameters
        -----
        pool:
            The process pool to perform the operation
        pool_size:
            The process pool size
        logger:
            The logging function taking a single str argument

        Returns
        -----
        An iterable of non-dominant solutions
        """
        raise NotImplementedError
