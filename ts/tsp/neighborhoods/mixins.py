from __future__ import annotations

from typing import Any, Dict, Protocol, Type, TYPE_CHECKING

if TYPE_CHECKING:
    from ..solutions import TSPPathSolution


if TYPE_CHECKING:
    class TSPNeighborhoodProtocol(Protocol):
        @property
        def cls(self) -> Type[TSPPathSolution]: ...
        @property
        def extras(self) -> Dict[Any, Any]: ...


class TSPNeighborhoodMixin:

    __slots__ = ()

    def __init__(self: TSPNeighborhoodProtocol, solution: TSPPathSolution, /) -> None:
        # super() should resolve to BaseNeighborhood
        super().__init__(solution)  # type: ignore
        self.extras["problem"] = solution.problem_name
        self.extras["distances"] = solution.distances

    def ensure_imported_data(self: TSPNeighborhoodProtocol) -> None:
        if self.cls.problem_name is None:
            self.cls.import_problem(self.extras["problem"], precalculated_distances=self.extras["distances"])
