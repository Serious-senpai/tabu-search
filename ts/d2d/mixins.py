from __future__ import annotations

from functools import total_ordering
from typing import Any, Optional, Sequence, Tuple, TYPE_CHECKING, overload

from ..abc import BaseCostComparison


__all__ = (
    "SolutionMetricsMixin",
)


@total_ordering
class MultiObjectiveWrapper:

    __slots__ = ("__inner",)
    if TYPE_CHECKING:
        __inner: Tuple[float, ...]

    @overload
    def __init__(self, *__elements: float) -> None: ...
    @overload
    def __init__(self, __sequence: Sequence[float]) -> None: ...

    def __init__(self, *args: Any) -> None:
        self.__inner = args if isinstance(args[0], float) else args[0]

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, MultiObjectiveWrapper):
            first_dominant = second_dominant = False
            for f, s in zip(self.__inner, other.__inner):
                if f > s:
                    first_dominant = True
                elif f < s:
                    second_dominant = True

            return first_dominant == second_dominant

        return NotImplemented

    def __lt__(self, other: Any) -> bool:
        if isinstance(other, MultiObjectiveWrapper):
            for f, s in zip(self.__inner, other.__inner):
                if f > s:
                    return False

            return True

        return NotImplemented


class SolutionMetricsMixin(BaseCostComparison):

    __slots__ = (
        "_cost",
        "drone_timespans",
        "drone_waiting_times",
        "technician_timespans",
        "technician_waiting_times",
    )
    if TYPE_CHECKING:
        _cost: Optional[MultiObjectiveWrapper]
        drone_timespans: Tuple[float, ...]
        drone_waiting_times: Tuple[Tuple[float, ...], ...]
        technician_timespans: Tuple[float, ...]
        technician_waiting_times: Tuple[float, ...]

    def __init__(
        self,
        *args: Any,
        drone_timespans: Tuple[float, ...],
        drone_waiting_times: Tuple[Tuple[float, ...], ...],
        technician_timespans: Tuple[float, ...],
        technician_waiting_times: Tuple[float, ...],
        **kwargs: Any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.drone_timespans = drone_timespans
        self.drone_waiting_times = drone_waiting_times
        self.technician_timespans = technician_timespans
        self.technician_waiting_times = technician_waiting_times

        self._cost = None

    def cost(self) -> MultiObjectiveWrapper:
        if self._cost is None:
            self._cost = MultiObjectiveWrapper(
                max(*self.drone_timespans, *self.technician_timespans),
                sum(sum(t) for t in self.drone_waiting_times) + sum(self.technician_waiting_times),
            )

        return self._cost
