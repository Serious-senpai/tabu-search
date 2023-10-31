from __future__ import annotations

from typing import Any, Optional, Tuple, TYPE_CHECKING

from ..abc import BaseMulticostComparison


__all__ = (
    "SolutionMetricsMixin",
)


class SolutionMetricsMixin(BaseMulticostComparison):

    __slots__ = (
        "_cost",
        "drone_timespans",
        "drone_waiting_times",
        "technician_timespans",
        "technician_waiting_times",
    )
    if TYPE_CHECKING:
        _cost: Optional[Tuple[float, float]]
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

    def cost(self) -> Tuple[float, float]:
        """The cost of the solution that this object represents."""
        if self._cost is None:
            self._cost = (
                max(*self.drone_timespans, *self.technician_timespans),
                sum(sum(t) for t in self.drone_waiting_times) + sum(self.technician_waiting_times),
            )

        return self._cost
