from __future__ import annotations

from typing import Any, Tuple, TYPE_CHECKING


__all__ = (
    "SupportConstraintsMixin",
)


class SupportConstraintsMixin:

    __slots__ = (
        "timespans",
        "waiting_durations",
    )
    if TYPE_CHECKING:
        timespans: Tuple[Tuple[float, ...], Tuple[float, ...]]
        waiting_durations: Tuple[Tuple[float, ...], Tuple[float, ...]]

    def __init__(
        self,
        *args: Any,
        timespans: Tuple[Tuple[float, ...], Tuple[float, ...]],
        waiting_durations: Tuple[Tuple[float, ...], Tuple[float, ...]],
        **kwargs: Any,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.timespans = timespans
        self.waiting_durations = waiting_durations
