from __future__ import annotations

from typing import Set, Tuple, TYPE_CHECKING, final

if TYPE_CHECKING:
    from typing_extensions import Self


__all__ = ("BaseMulticostComparison",)


class BaseMulticostComparison:

    __slots__ = ()

    def cost(self) -> Tuple[float, ...]:
        """The cost of this object

        Subclasses must implement this.
        """
        raise NotImplementedError

    @final
    def dominate(self, other: Self) -> bool:
        """Whether this object dominates another one"""
        return all(f <= s for f, s in zip(self.cost(), other.cost()))

    @final
    def add_to_pareto_set(self, __s: Set[Self], /) -> bool:
        """Add this object to the provided Pareto set.

        Objects which are currently in the set, but are dominated by this one, will be removed.

        If this object isn't dominated by any objects in the specified set, add it to the set.

        Returns
        -----
        Whether this object was added to the provided set.
        """
        to_remove = set(o for o in __s if self.dominate(o))
        for item in to_remove:
            __s.remove(item)

        if not any(o.dominate(self) for o in __s):
            __s.add(self)
            return True

        return False
