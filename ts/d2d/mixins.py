from __future__ import annotations

from typing import Any, Final, Optional, Tuple, TYPE_CHECKING

from ..abc import BaseMulticostComparison


__all__ = (
    "SolutionMetricsMixin",
)


class SolutionMetricsMixin(BaseMulticostComparison):

    __slots__ = (
        "_cost",
        "_fine",
        "fine_coefficient",
        "drone_timespans",
        "drone_waiting_times",
        "technician_timespans",
        "technician_waiting_times",
    )
    if TYPE_CHECKING:
        _cost: Optional[Tuple[float, float]]
        _fine: float
        fine_coefficient: float
        drone_timespans: Final[Tuple[float, ...]]
        drone_waiting_times: Final[Tuple[Tuple[float, ...], ...]]
        technician_timespans: Final[Tuple[float, ...]]
        technician_waiting_times: Final[Tuple[float, ...]]

    def __init__(
        self,
        *args: Any,
        drone_timespans: Tuple[float, ...],
        drone_waiting_times: Tuple[Tuple[float, ...], ...],
        technician_timespans: Tuple[float, ...],
        technician_waiting_times: Tuple[float, ...],
        fine: float = 0.0,
        fine_coefficient: float = 100.0,
        **kwargs: Any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.drone_timespans = drone_timespans
        self.drone_waiting_times = drone_waiting_times
        self.technician_timespans = technician_timespans
        self.technician_waiting_times = technician_waiting_times

        self._cost = None
        self._fine = fine
        self.fine_coefficient = fine_coefficient

    def bump_fine_coefficient(self) -> None:
        self.fine_coefficient *= 10.0

    def cost(self) -> Tuple[float, float]:
        """The cost of the solution that this object represents."""
        if self._cost is None:
            self._cost = (
                max(*self.drone_timespans, *self.technician_timespans),
                sum(sum(t) for t in self.drone_waiting_times) + sum(self.technician_waiting_times),
            )

        return (self._cost[0] + self.fine_coefficient * self._fine, self._cost[1] + self.fine_coefficient * self._fine)
