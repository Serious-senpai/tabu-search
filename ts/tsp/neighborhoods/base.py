from __future__ import annotations

from typing import Generic, TypeVar, TYPE_CHECKING

from ...abc import SingleObjectiveNeighborhood
if TYPE_CHECKING:
    from ..solutions import TSPPathSolution


__all__ = ("TSPBaseNeighborhood",)


_T = TypeVar("_T")
if TYPE_CHECKING:
    class TSPBaseNeighborhood(SingleObjectiveNeighborhood[TSPPathSolution, _T]):
        __slots__ = ()
        def __init__(self, solution: TSPPathSolution, /) -> None: ...
        def ensure_imported_data(self) -> None: ...

else:
    class TSPBaseNeighborhood(SingleObjectiveNeighborhood, Generic[_T]):

        __slots__ = ()

        def __init__(self, solution: TSPPathSolution, /) -> None:
            super().__init__(solution)
            self.extras["problem"] = solution.problem_name
            self.extras["distances"] = solution.distances

        def ensure_imported_data(self) -> None:
            if self.cls.problem_name is None:
                self.cls.import_problem(self.extras["problem"], precalculated_distances=self.extras["distances"])
