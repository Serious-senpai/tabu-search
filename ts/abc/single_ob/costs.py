from __future__ import annotations

from functools import total_ordering
from typing import Any, final


__all__ = ("BaseCostComparison",)


@total_ordering
class BaseCostComparison:
    """Base class for objects holding a real-valued number as their costs"""

    __slots__ = ()

    def cost(self) -> float:
        """The cost of this object

        Subclasses must implement this.
        """
        raise NotImplementedError

    @final
    def __eq__(self, other: Any) -> bool:
        if isinstance(other, self.__class__):
            return self.cost() == other.cost()

        return NotImplemented

    @final
    def __lt__(self, other: Any) -> bool:
        if isinstance(other, self.__class__):
            return self.cost() < other.cost()

        return NotImplemented
