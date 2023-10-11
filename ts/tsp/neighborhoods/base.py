from __future__ import annotations

from typing import ClassVar, Deque, Generic, Set, TypeVar, TYPE_CHECKING

from ...abc import BaseNeighborhood
if TYPE_CHECKING:
    from ..solutions import TSPPathSolution


_T = TypeVar("_T")


class BasePathNeighborhood(BaseNeighborhood["TSPPathSolution"], Generic[_T]):

    __slots__ = ()
    if TYPE_CHECKING:
        _maxlen: ClassVar[int]
        _tabu_list: ClassVar[Deque[_T]]  # type: ignore
        _tabu_set: ClassVar[Set[_T]]  # type: ignore

    def __init__(self, solution: TSPPathSolution, /) -> None:
        super().__init__(solution)
        self.extras["problem"] = solution.problem_name
        self.extras["distances"] = solution.distances

    def _ensure_imported_data(self) -> None:
        if self.cls.problem_name is None:
            self.cls.import_problem(self.extras["problem"], precalculated_distances=self.extras["distances"])

    @classmethod
    def add_to_tabu(cls, target: _T) -> None:
        cls._tabu_set.add(target)
        cls._tabu_list.append(target)
        cls.remove_from_tabu()

    @classmethod
    def remove_from_tabu(cls) -> None:
        while len(cls._tabu_set) > cls._maxlen:
            try:
                cls._tabu_set.remove(cls._tabu_list.popleft())
            except KeyError:
                pass

    @classmethod
    def reset_tabu(cls, *, maxlen: int = 100) -> None:
        cls._maxlen = maxlen
        cls.remove_from_tabu()
