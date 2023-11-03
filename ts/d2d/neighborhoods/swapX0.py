from __future__ import annotations
from multiprocessing import pool as p
from typing import Iterable, Set, TYPE_CHECKING
from typing import Dict, Iterable, List, Set, Tuple, TYPE_CHECKING
import itertools
from .mixins import D2DNeighborhoodMixin
from .factory import SolutionFactory
from ...abc import MultiObjectiveNeighborhood
from ...bundle import IPCBundle
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

        # swap Technician - Technician

        bundles: List[IPCBundle[Swappoint, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
        bundles_iter = itertools.cycle(bundles)

        for pair in itertools.permutations(range(solution.technicians_count), 2):
            x = next(bundles_iter)
            x.data.append(pair)  # type: ignore

        return results

    @staticmethod
    def swap_technician_technician(bundle: IPCBundle[Swappoint, List[Tuple[int, int]]]) -> Set[Tuple[SolutionFactory, Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[int, int]] = {}

        for i, j in bundle.data:
            i_path = solution.technician_paths[i]
            j_path = solution.technician_paths[j]

            for point_i in range(1, len(i_path) - neighborhood.length):
                for location_j in range(1, len(j_path) - 1):
                    pi = list(solution.technician_paths[i])
                    pj = list(solution.technician_paths[j])
                    pj[location_j:location_j] = solution.technician_paths[i][point_i:point_i + neighborhood.length]
                    pi[point_i:point_i + neighborhood.length] = []

                    first_arrival_timestamps = solution.calculate_technician_arrival_timestamps(pi)
                    second_arrival_timestamps = solution.calculate_technician_arrival_timestamps(pj)

                    _technician_timespans = list(solution.technician_timespans)
                    _technician_timespans[i] = first_arrival_timestamps[-1]
                    _technician_timespans[j] = second_arrival_timestamps[-1]

                    _technician_total_waiting_times = list(solution.technician_waiting_times)
                    _technician_total_waiting_times[i] = solution.calculate_technician_total_waiting_time(pi, arrival_timestamps=first_arrival_timestamps)
                    _technician_total_waiting_times[j] = solution.calculate_technician_total_waiting_time(pj, arrival_timestamps=second_arrival_timestamps)

                    factory = SolutionFactory(
                        update_technicians=((i, tuple(pi)), (j, tuple(pj))),
                        drone_timespans=solution.drone_timespans,
                        drone_waiting_times=solution.drone_waiting_times,
                        technician_timespans=tuple(_technician_timespans),
                        technician_waiting_times=tuple(_technician_total_waiting_times),
                    )
                    factory.add_to_pareto_set(results)
                    factory = SolutionFactory(
                        update_technicians=((i, tuple(pi)), (j, tuple(pj))),
                        drone_timespans=solution.drone_timespans,
                        drone_waiting_times=solution.drone_waiting_times,
                        technician_timespans=tuple(_technician_timespans),
                        technician_waiting_times=tuple(_technician_total_waiting_times),
                    )

                    pair = (i_path[point_i], j_path[location_j])
                    swaps_mapping[factory] = (min(pair), max(pair))
                    factory.add_to_pareto_set(results)

        return set((r, swaps_mapping[r]) for r in results)
