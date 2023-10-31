from multiprocessing import pool as p
from typing import Iterable, Set, TYPE_CHECKING

from .mixins import D2DNeighborhoodMixin
from ...abc import MultiObjectiveNeighborhood
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


if TYPE_CHECKING:
    _BaseNeighborhood = MultiObjectiveNeighborhood[D2DPathSolution, ]
else:
    _BaseNeighborhood = MultiObjectiveNeighborhood



class Swappoint(D2DNeighborhoodMixin, _BaseNeighborhood):

    __slots__ = (
        "length",
    )

    if TYPE_CHECKING:
        length: int

    def __init__(self, solution: D2DPathSolution, *, length: int) -> None:
        super().__init__(solution)
    
    
    
    def find_best_candidates(self, *, pool: p.Pool, pool_size: int) -> Iterable[D2DPathSolution]:
        solution = self._solution
        results: Set[OperationResult] = set()
        swaps_mapping: Dict[OperationResult, Tuple[int, int]] = {}
        
        def callback(collected: Iterable[Set[Tuple[OperationResult, Tuple[int, int]]]]) -> None:
            for s in collected:
                for result, pair in s:
                    swaps_mapping[result] = pair
                    result.add_to_pareto_set(results)
        