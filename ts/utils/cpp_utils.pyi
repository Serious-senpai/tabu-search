from __future__ import annotations

from typing import List, Optional, Sequence, Tuple


__all__ = (
    "tsp_solver",
)


def tsp_solver(
    cities: Sequence[Tuple[float, float]],
    *,
    first: int = 0,
    heuristic_hint: Optional[Sequence[int]] = None,
) -> Tuple[float, List[int]]: ...
