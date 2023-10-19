from __future__ import annotations

from typing import Any, Tuple, TYPE_CHECKING


__all__ = (
    "SolutionMetricsMixin",
)


class SolutionMetricsMixin:

    __slots__ = (
        "timespan",
        "total_waiting_time",
    )
    if TYPE_CHECKING:
        timespan: float
        total_waiting_time: float

    def __init__(
        self,
        *args: Any,
        timespan: float,
        total_waiting_time: float,
        **kwargs: Any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.timespan = timespan
        self.total_waiting_time = total_waiting_time
