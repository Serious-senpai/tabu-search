from __future__ import annotations
from multiprocessing import pool as p
from typing import Iterable, Set, TYPE_CHECKING
from copy import deepcopy
from typing import Dict, Iterable, List, Set, Tuple, TYPE_CHECKING
import itertools
from .mixins import D2DNeighborhoodMixin
from .results import OperationResult
from ..config import DroneEnergyConsumptionMode
from ..errors import NeighborhoodException
from ...abc import MultiObjectiveNeighborhood
from ...bundle import IPCBundle
from .mixins import D2DNeighborhoodMixin
from ...abc import MultiObjectiveNeighborhood
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("Swappoint",)
if TYPE_CHECKING:
    _BaseNeighborhood = MultiObjectiveNeighborhood[D2DPathSolution, Tuple[int, int]]
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
        self.length = length
        
        
    
    
    def find_best_candidates(self, *, pool: p.Pool, pool_size: int) -> Iterable[D2DPathSolution]:
        solution = self._solution
        results: Set[OperationResult] = set()
        swaps_mapping: Dict[OperationResult, Tuple[int, int]] = {}
        
        #swap Technician - Technician
        
        for i, j in itertools.permutations(range(solution.technicians_count), 2):
        
            i_path = solution.technician_paths[i]
            j_path = solution.technician_paths[j]
            
            for point_i in range(1, len(i_path) - self.length):
                for location_j in range(1, len(j_path) - 1):
                    technician_path = list(solution.technician_paths)
                    technician_path[i] = list(technician_path[i])
                    technician_path[j] = list(technician_path[j])
                    technician_path[j][location_j:location_j] = technician_path[i][point_i:point_i + self.length]
                    technician_path[i][point_i:point_i + self.length] = []
                    
                    s=self.cls(
                        drone_paths=solution.drone_paths,
                        technician_paths=technician_path,
                        drone_config_mapping=solution.drone_config_mapping,
                    )    
                    s.add_to_pareto_set(results)
                    
        return results    
                    