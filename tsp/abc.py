from __future__ import annotations

from collections import deque
from functools import total_ordering
from typing import Any, Generic, Iterable, TypeVar, TYPE_CHECKING

if TYPE_CHECKING:
    from typing_extensions import Self


__all__ = (
    "BaseSolution",
    "BaseNeighborHood",
)


@total_ordering
class BaseSolution:
    """Base class for objects holding a solution to the problem"""

    __slots__ = ()

    def cost(self) -> float:
        """Calculate the cost for this solution"""
        raise NotImplementedError

    def get_neighborhoods(self) -> Iterable[BaseNeighborHood[Self]]:
        """Returns all neighborhoods of the current solution"""
        raise NotImplementedError

    def post_optimization(self) -> Self:
        """Perform post-optimization for this solution

        The default implementation does nothing.
        """
        return self

    @classmethod
    def tabu_search(cls, *, initial: Self, iterations_count: int = 50) -> Self:
        result = initial
        tabu = deque(maxlen=20)
        for _ in range(iterations_count):
            best_candidate = None
            for neighborhood in result.get_neighborhoods():
                for candidate in neighborhood.generate():
                    if best_candidate not in tabu:
                        if best_candidate is None:
                            best_candidate = candidate
                        else:
                            best_candidate = min(best_candidate, candidate)

            if best_candidate is None:
                break

            result = min(result, best_candidate)
            tabu.append(best_candidate)

        return result.post_optimization()

    def __hash__(self) -> int:
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} hash={self.__hash__()}>"

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, self.__class__):
            return self.cost() == other.cost()

        return NotImplemented

    def __lt__(self, other: Any) -> bool:
        if isinstance(other, self.__class__):
            return self.cost() < other.cost()

        return NotImplemented


T = TypeVar("T", bound=BaseSolution)


class BaseNeighborHood(Generic[T]):
    """Base class for generating neighborhood of a solution"""

    __slots__ = (
        "_solution",
    )
    if TYPE_CHECKING:
        _solution: T

    def __init__(self, solution: T, /) -> None:
        self._solution = solution

    def generate(self) -> Iterable[T]:
        """Return all feasible solutions in the neighborhood of the current solution"""
        raise NotImplementedError
