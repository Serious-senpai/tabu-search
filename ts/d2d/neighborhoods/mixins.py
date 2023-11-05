from __future__ import annotations

from typing import Any, Dict, Protocol, Type, TYPE_CHECKING

if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("D2DNeighborhoodMixin",)


if TYPE_CHECKING:
    class D2DNeighborhoodProtocol(Protocol):
        @property
        def cls(self) -> Type[D2DPathSolution]: ...
        @property
        def extras(self) -> Dict[Any, Any]: ...


class D2DNeighborhoodMixin:

    __slots__ = ()

    def __init__(self: D2DNeighborhoodProtocol, solution: D2DPathSolution, /) -> None:
        # super() should resolve to MultiObjectiveNeighborhood
        super().__init__(solution)  # type: ignore
        self.extras["problem"] = solution.problem
        self.extras["drone_config_mapping"] = solution.drone_config_mapping
        self.extras["energy_mode"] = solution.energy_mode

    def ensure_imported_data(self: D2DNeighborhoodProtocol) -> None:
        if self.cls.problem != self.extras["problem"]:
            self.cls.import_problem(
                self.extras["problem"],
                drone_config_mapping=self.extras["drone_config_mapping"],
                energy_mode=self.extras["energy_mode"],
            )
