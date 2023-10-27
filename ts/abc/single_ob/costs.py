from __future__ import annotations

from functools import total_ordering
from typing import Any, TYPE_CHECKING, final

if TYPE_CHECKING:
    from ..types import SupportsCost


__all__ = ("BaseCostComparison",)


@total_ordering
class BaseCostComparison:

    __slots__ = ()

    def cost(self) -> SupportsCost:
        """The cost of this object"""
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
