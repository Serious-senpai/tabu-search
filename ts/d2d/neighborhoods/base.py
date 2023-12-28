from __future__ import annotations

from typing import TypeVar, TYPE_CHECKING

from ...abc import MultiObjectiveNeighborhood
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("D2DBaseNeighborhood",)
_T = TypeVar("_T")
if TYPE_CHECKING:
    _D2DPathSolution = D2DPathSolution
else:
    _D2DPathSolution = object


class D2DBaseNeighborhood(MultiObjectiveNeighborhood[_D2DPathSolution, _T]):
    """Base class for neighborhood of the D2D problem. When working in a subprocess, remember to call
    `D2DBaseNeighborhood.ensure_imported_data` first.
    """

    __slots__ = ()

    def __init__(self, solution: D2DPathSolution, /) -> None:
        super().__init__(solution)
        self.extras["problem"] = solution.problem
        self.extras["drone_config"] = solution.drone_config
        self.extras["energy_mode"] = solution.energy_mode
        self.extras["precalculated_distances"] = solution.distances

    def ensure_imported_data(self) -> None:
        if self.cls.problem != self.extras["problem"]:
            self.cls.import_problem(
                self.extras["problem"],
                drone_config=self.extras["drone_config"],
                energy_mode=self.extras["energy_mode"],
                precalculated_distances=self.extras["precalculated_distances"],
            )
