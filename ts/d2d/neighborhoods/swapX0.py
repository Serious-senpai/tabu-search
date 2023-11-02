from __future__ import annotations
from multiprocessing import pool as p
from typing import Iterable, Set, TYPE_CHECKING
from copy import deepcopy
from typing import Dict, Iterable, List, Set, Tuple, TYPE_CHECKING
import itertools
import functools
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
        results: Set[D2DPathSolution] = set()
        swaps_mapping: Dict[D2DPathSolution, Tuple[int, int]] = {}
        
        #swap Technician - Technician
        
        for i, j in itertools.permutations(range(solution.technicians_count), 2):
        
            i_path = solution.technician_paths[i]
            j_path = solution.technician_paths[j]
            
            for point_i in range(1, len(i_path) - self.length):
                for location_j in range(1, len(j_path) - 1):
                    technician_path = list(solution.technician_paths)
                    pi = list(technician_path[i])
                    pj = list(technician_path[j])
                    pj[location_j:location_j] = technician_path[i][point_i:point_i + self.length]
                    pi[point_i:point_i + self.length] = []
                    technician_path[i] = tuple(pi)
                    technician_path[j] = tuple(pj)
                    s=self.cls(
                        drone_paths=solution.drone_paths,
                        technician_paths=tuple(technician_path),
                        drone_config_mapping=solution.drone_config_mapping,
                    )    
                    s.add_to_pareto_set(results)
                    
        return results    
                    
    @staticmethod
    def swap_technician_technician(bundle: IPCBundle[Swappoint, List[Tuple[int, int]]]) -> Set[Tuple[OperationResult, Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        first_length = neighborhood.length
        second_length = neighborhood.length

        # Don't alter the variables without a prefix underscore, edit their copies instead
        technician_paths = list(list(path) for path in solution.technician_paths)

        results: Set[OperationResult] = set()
        swaps_mapping: Dict[OperationResult, Tuple[int, int]] = {}
        for first, second in bundle.data:
            first_path = technician_paths[first]
            second_path = technician_paths[second]

            for first_start, second_start in itertools.product(
                range(1, len(first_path) - first_length),
                range(1, len(second_path) - second_length),
            ):
                _first_path = first_path.copy()
                _second_path = second_path.copy()

                _first_path[first_start:first_start + first_length] = second_path[second_start:second_start + second_length]
                _second_path[second_start:second_start + second_length] = first_path[first_start:first_start + first_length]

                first_arrival_timestamps = solution.calculate_technician_arrival_timestamps(_first_path)
                second_arrival_timestamps = solution.calculate_technician_arrival_timestamps(_second_path)

                _technician_timespans = list(solution.technician_timespans)
                _technician_timespans[first] = first_arrival_timestamps[-1]
                _technician_timespans[second] = second_arrival_timestamps[-1]

                _technician_total_waiting_times = list(solution.technician_waiting_times)
                _technician_total_waiting_times[first] = solution.calculate_technician_total_waiting_time(_first_path, arrival_timestamps=first_arrival_timestamps)
                _technician_total_waiting_times[second] = solution.calculate_technician_total_waiting_time(_second_path, arrival_timestamps=second_arrival_timestamps)

                _technician_paths = deepcopy(technician_paths)
                _technician_paths[first] = _first_path
                _technician_paths[second] = _second_path

                operation_result = OperationResult(
                    factory=functools.partial(
                        neighborhood.cls,
                        drone_paths=solution.drone_paths,
                        technician_paths=tuple(tuple(path) for path in _technician_paths),
                        drone_config_mapping=solution.drone_config_mapping,
                    ),
                    drone_timespans=solution.drone_timespans,
                    drone_waiting_times=solution.drone_waiting_times,
                    technician_timespans=tuple(_technician_timespans),
                    technician_waiting_times=tuple(_technician_total_waiting_times),
                )

                pair = (first_path[first_start], second_path[second_start])
                swaps_mapping[operation_result] = (min(pair), max(pair))
                operation_result.add_to_pareto_set(results)

        return set((r, swaps_mapping[r]) for r in results)