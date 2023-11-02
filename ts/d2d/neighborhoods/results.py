from __future__ import annotations

from typing import Callable, Tuple, TYPE_CHECKING

from ..mixins import SolutionMetricsMixin
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("OperationResult",)


class OperationResult(SolutionMetricsMixin):

    __slots__ = (
        "__factory",
    )
    if TYPE_CHECKING:
        __factory: Callable[..., D2DPathSolution]

    def __init__(
        self,
        *,
        factory: Callable[..., D2DPathSolution],
        drone_timespans: Tuple[float, ...],
        drone_waiting_times: Tuple[Tuple[float, ...], ...],
        technician_timespans: Tuple[float, ...],
        technician_waiting_times: Tuple[float, ...],
    ) -> None:
        super().__init__(
            drone_timespans=drone_timespans,
            drone_waiting_times=drone_waiting_times,
            technician_timespans=technician_timespans,
            technician_waiting_times=technician_waiting_times,
        )
        self.__factory = factory

    def to_solution(self) -> D2DPathSolution:
        return self.__factory(
            drone_timespans=self.drone_timespans,
            drone_waiting_times=self.drone_waiting_times,
            technician_timespans=self.technician_timespans,
            technician_waiting_times=self.technician_waiting_times,
        )

    def __hash__(self) -> int:
        return hash(self.__factory)
