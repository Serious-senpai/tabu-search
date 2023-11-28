from __future__ import annotations

from typing import Generic, TypeVar, TYPE_CHECKING

from ...abc import MultiObjectiveNeighborhood
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("D2DBaseNeighborhood",)


_T = TypeVar("_T")
if TYPE_CHECKING:
    class D2DBaseNeighborhood(MultiObjectiveNeighborhood[D2DPathSolution, _T]):
        """Base class for neighborhood of the D2D problem. When working in a subprocess, remember to call
        `D2DBaseNeighborhood.ensure_imported_data` first.
        """

        __slots__ = ()
        def __init__(self, solution: D2DPathSolution, /) -> None: ...
        def ensure_imported_data(self) -> None: ...

else:
    class D2DBaseNeighborhood(MultiObjectiveNeighborhood, Generic[_T]):

        __slots__ = ()

        def __init__(self, solution: D2DPathSolution, /) -> None:
            super().__init__(solution)
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
