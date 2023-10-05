from __future__ import annotations

from functools import total_ordering
from multiprocessing import Pool
from typing import Any, Dict, Generic, Iterable, Optional, TypeVar, TYPE_CHECKING

from tqdm import tqdm
if TYPE_CHECKING:
    from typing_extensions import Self


__all__ = (
    "BaseSolution",
    "BaseNeighborhood",
)


@total_ordering
class BaseSolution:
    """Base class for objects holding a solution to the problem"""

    __slots__ = ()

    def cost(self) -> float:
        """Calculate the cost for this solution"""
        raise NotImplementedError

    def get_neighborhoods(self) -> Iterable[BaseNeighborhood[Self]]:
        """Returns all neighborhoods of the current solution"""
        raise NotImplementedError

    def post_optimization(self) -> Self:
        """Perform post-optimization for this solution

        The default implementation does nothing.
        """
        return self

    @classmethod
    def initial(cls) -> Self:
        """Generate the initial solution for tabu search"""
        raise NotImplementedError

    @classmethod
    def tabu_search(cls, *, iterations_count: int = 50, use_tqdm: bool = True) -> Self:
        result = cls.initial()
        iterations = range(iterations_count)
        if use_tqdm:
            iterations = tqdm(iterations, ascii=" █")

        with Pool() as pool:
            for _ in iterations:
                best_candidates = pool.map(BaseNeighborhood.static_find_best_candidate, result.get_neighborhoods())
                best_candidate: Optional[Self] = None
                for candidate in best_candidates:
                    if best_candidate is None or candidate < best_candidate:
                        best_candidate = candidate

                if best_candidate is None:
                    break

                result = min(result, best_candidate)

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


class BaseNeighborhood(Generic[T]):
    """Base class for generating neighborhood of a solution"""

    __slots__ = (
        "_solution",
        "extras",
    )
    if TYPE_CHECKING:
        _solution: T
        extras: Dict[Any, Any]

    def __init__(self, solution: T, /) -> None:
        self._solution = solution
        self.extras = {}

    def find_best_candidate(self) -> Optional[T]:
        """Find the best candidate solution within the neighborhood of the current one.

        Subclasses should implement the tabu logic internally.
        """
        raise NotImplementedError

    @staticmethod
    def static_find_best_candidate(neighborhood: BaseNeighborhood[T]) -> Optional[T]:
        return neighborhood.find_best_candidate()
