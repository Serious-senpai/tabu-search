from __future__ import annotations

from typing import TypeVar, TYPE_CHECKING

from ...abc import MultiObjectiveNeighborhood
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("D2DBaseNeighborhood",)


_T = TypeVar("_T")
if TYPE_CHECKING:
    _BaseNeighborhood = MultiObjectiveNeighborhood[D2DPathSolution, _T]
else:
    _BaseNeighborhood = MultiObjectiveNeighborhood


class D2DBaseNeighborhood(_BaseNeighborhood[_T]):

    __slots__ = ()

    def __init__(self, solution: D2DPathSolution, /) -> None:
        super().__init__(solution)  # type: ignore
        self.extras["problem"] = solution.problem
        self.extras["drone_config_mapping"] = solution.drone_config_mapping
        self.extras["energy_mode"] = solution.energy_mode

    def ensure_imported_data(self) -> None:
        if self.cls.problem != self.extras["problem"]:
            self.cls.import_problem(
                self.extras["problem"],
                drone_config_mapping=self.extras["drone_config_mapping"],
                energy_mode=self.extras["energy_mode"],
            )
