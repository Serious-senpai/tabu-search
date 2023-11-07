from __future__ import annotations

from multiprocessing import pool
from typing import Any, Callable, Optional, TypeVar

from .solutions import SingleObjectiveSolution
from ..bases import BaseNeighborhood


__all__ = ("SingleObjectiveNeighborhood",)
_SingleST = TypeVar("_SingleST", bound=SingleObjectiveSolution)
_TT = TypeVar("_TT")


class SingleObjectiveNeighborhood(BaseNeighborhood[_SingleST, _TT]):
    """Base class for neighborhoods of a solution to a single-objective optimization problem"""

    def find_best_candidate(self, *, pool: pool.Pool, pool_size: int, logger: Optional[Callable[[str], Any]]) -> Optional[_SingleST]:
        """Find the best candidate solution within the neighborhood of the current one.

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
        The best candidate solution, or `None` if there are no candidates
        """
        raise NotImplementedError
