from __future__ import annotations

import random
from collections import deque
from functools import total_ordering
from multiprocessing import Pool, pool
from typing import Any, ClassVar, Deque, Dict, Generic, Optional, Set, Tuple, Type, TypeVar, Union, TYPE_CHECKING

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

    def get_neighborhoods(self) -> Tuple[BaseNeighborhood[Self, Any], ...]:
        """Returns all neighborhoods of the current solution"""
        raise NotImplementedError

    def shuffle(self, *, use_tqdm: bool) -> Self:
        """Shuffle the current solution

        After a certain (or random) number of iterations and the solution does not improve,
        shuffle it randomly.

        The default implementation does nothing.

        Parameters
        -----
        use_tqdm: `bool`
            Whether to display the progress bar
        """
        return self

    def post_optimization(self, *, pool: pool.Pool, pool_size: int, use_tqdm: bool) -> Self:
        """Perform post-optimization for this solution

        The default implementation does nothing.

        Parameters
        -----
        pool: `pool.Pool`
            The process pool to perform post-optimization
        pool_size: `int`
            The process pool size
        use_tqdm: `bool`
            Whether to display the progress bar
        """
        return self

    @classmethod
    def initial(cls) -> Self:
        """Generate the initial solution for tabu search"""
        raise NotImplementedError

    @classmethod
    def tabu_search(cls, *, pool_size: int, iterations_count: int, use_tqdm: bool, shuffle_after: int) -> Self:
        result = current = cls.initial()
        iterations: Union[range, tqdm[int]] = range(iterations_count)
        if use_tqdm:
            iterations = tqdm(iterations, ascii=" █")

        with Pool(pool_size) as pool:
            last_improved = 0
            for iteration in iterations:
                if isinstance(iterations, tqdm):
                    iterations.set_description_str(f"Tabu search ({current.cost()}/{result.cost()})")

                neighborhoods = current.get_neighborhoods()
                best_candidate = random.choice(neighborhoods).find_best_candidate(pool=pool, pool_size=pool_size)
                if best_candidate is None:
                    break

                if best_candidate < current:
                    last_improved = iteration

                current = best_candidate
                result = min(result, current)

                if iteration - last_improved >= shuffle_after:
                    current = current.shuffle(use_tqdm=use_tqdm)
                    last_improved = iteration

            return result.post_optimization(pool=pool, pool_size=pool_size, use_tqdm=use_tqdm)

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


_ST = TypeVar("_ST", bound=BaseSolution)
_TT = TypeVar("_TT")


class BaseNeighborhood(Generic[_ST, _TT]):
    """Base class for generating neighborhood of a solution"""

    __slots__ = (
        "_solution",
        "cls",
        "extras",
    )
    if TYPE_CHECKING:
        _solution: _ST
        cls: Type[_ST]
        extras: Dict[Any, Any]

        _maxlen: ClassVar[int]
        _tabu_list: ClassVar[Deque[_TT]]  # type: ignore
        _tabu_set: ClassVar[Set[_TT]]  # type: ignore

    def __init__(self, solution: _ST, /) -> None:
        self._solution = solution
        self.cls = type(solution)
        self.extras = {}

    def __init_subclass__(cls, *args: Any, **kwargs: Any) -> None:
        super().__init_subclass__(*args, **kwargs)
        cls._maxlen = 10
        cls._tabu_list = deque()
        cls._tabu_set = set()

    def find_best_candidate(self, *, pool: pool.Pool, pool_size: int) -> Optional[_ST]:
        """Find the best candidate solution within the neighborhood of the current one.

        Parameters
        -----
        pool: `pool.Pool`
            The process pool to perform the operation
        pool_size: `int`
            The process pool size
        """
        raise NotImplementedError

    @classmethod
    def add_to_tabu(cls, target: _TT) -> None:
        cls._tabu_set.add(target)
        cls._tabu_list.append(target)
        cls.remove_from_tabu()

    @classmethod
    def remove_from_tabu(cls) -> None:
        while len(cls._tabu_set) > cls._maxlen:
            try:
                cls._tabu_set.remove(cls._tabu_list.popleft())
            except KeyError:
                pass

    @classmethod
    def reset_tabu(cls, *, maxlen: int = 10) -> None:
        cls._maxlen = maxlen
        cls.remove_from_tabu()
