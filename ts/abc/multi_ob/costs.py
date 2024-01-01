from __future__ import annotations

import itertools
from typing import Dict, Iterable, Iterator, Final, Generic, Optional, Set, Tuple, TypeVar, Union, TYPE_CHECKING, final

if TYPE_CHECKING:
    from typing_extensions import Self

from ...utils import cost_dominate


__all__ = ("BaseMulticostComparison", "ParetoSet")


class BaseMulticostComparison:
    """Base class for objects holding a vector as their costs"""

    __slots__ = ()

    def cost(self) -> Tuple[float, ...]:
        """The cost of this object

        Subclasses must implement this.
        """
        raise NotImplementedError

    @final
    def dominate(self, other: Self) -> bool:
        """Whether this object dominates another one"""
        return cost_dominate(self.cost(), other.cost())

    def add_to_pareto_set(self, __s: Union[Set[Self], ParetoSet[Self]], /) -> Tuple[bool, Set[Self]]:
        """Add this object to the provided Pareto set.

        Objects which are currently in the set, but are dominated by this one, will be removed.

        If this object isn't dominated by any objects in the specified set, add it to the set.

        Returns
        -----
        A pair of 2 values:
        - Whether this object was added to the provided set
        - The objects removed from the set
        """
        if isinstance(__s, ParetoSet):
            return __s.add(self)

        to_remove = set(o for o in __s if self.dominate(o))
        for item in to_remove:
            __s.remove(item)

        if not any(o.dominate(self) for o in __s):
            __s.add(self)
            return True, to_remove

        return False, to_remove


_ST = TypeVar("_ST", bound=BaseMulticostComparison)


class ParetoSet(Generic[_ST]):

    __slots__ = (
        "__cost_to_solutions",
        "__length",
    )
    if TYPE_CHECKING:
        __length: int

    def __init__(self, initial: Optional[Iterable[_ST]] = None, /) -> None:
        self.__cost_to_solutions: Final[Dict[Tuple[float, ...], Set[_ST]]] = {}
        self.__length = 0
        if initial is not None:
            for s in initial:
                self.add(s)

    def keys(self) -> Iterable[Tuple[float, ...]]:
        return self.__cost_to_solutions.keys()

    def counter(self) -> Dict[Tuple[float, ...], int]:
        """Return a counter of the costs of the solutions in this set"""
        return {k: len(v) for k, v in self.__cost_to_solutions.items()}

    def add(self, __s: _ST, /) -> Tuple[bool, Set[_ST]]:
        __s_cost = tuple(round(c, 4) for c in __s.cost())
        try:
            if __s not in self.__cost_to_solutions[__s_cost]:
                self.__length += 1
                self.__cost_to_solutions[__s_cost].add(__s)

            return True, set()

        except KeyError:
            removed_costs = set(c for c in self.__cost_to_solutions.keys() if cost_dominate(__s_cost, c))
            removed = set(itertools.chain(*[self.__cost_to_solutions[c] for c in removed_costs]))
            for cost in removed_costs:
                del self.__cost_to_solutions[cost]

            self.__length -= len(removed)

            if any(cost_dominate(c, __s_cost) for c in self.__cost_to_solutions.keys()):
                return False, removed

            self.__cost_to_solutions[__s_cost] = {__s}
            self.__length += 1
            return True, removed

    def __len__(self) -> int:
        return self.__length

    def __iter__(self) -> Iterator[_ST]:
        return itertools.chain(*self.__cost_to_solutions.values())
