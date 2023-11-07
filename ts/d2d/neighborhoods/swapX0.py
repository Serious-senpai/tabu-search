from __future__ import annotations

import itertools
from multiprocessing import pool as p
from typing import Dict, Iterable, List, Set, Tuple, TYPE_CHECKING

from .factory import SolutionFactory
from .mixins import D2DNeighborhoodMixin
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
        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[int, int]] = {}

        # swap Technician - Technician

        bundles: List[IPCBundle[Swappoint, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
        bundles_iter = itertools.cycle(bundles)

        for pair in itertools.permutations(range(solution.technicians_count), 2):
            x = next(bundles_iter)
            x.data.append(pair)  # type: ignore

        for candidates in pool.map(self.swap_technician_technician, bundles):
            for result, pair in candidates:
                if result.add_to_pareto_set(results):
                    swaps_mapping[result] = pair

        for result in results:
            pair = swaps_mapping[result]
            s = result.from_solution(solution)
            if pair in self.tabu_set:
                s.to_propagate = False

            else:
                self.add_to_tabu(pair)

            yield s

        # swap Drone - Drone
        for pair in itertools.permutations(range(solution.drones_count), 2):
            x = next(bundles_iter)
            x.data.append(pair)  # type: ignore

        for candidates in pool.map(self.swap_drone_drone, bundles):
            for result, pair in candidates:
                if result.add_to_pareto_set(results):
                    swaps_mapping[result] = pair

        for result in results:
            pair = swaps_mapping[result]
            s = result.from_solution(solution)
            if pair in self.tabu_set:
                s.to_propagate = False

            else:
                self.add_to_tabu(pair)

            yield s

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

    @staticmethod
    def swap_drone_drone(bundle: IPCBundle[Swappoint, List[Tuple[int, int]]]) -> Set[Tuple[SolutionFactory, Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[int, int]] = {}

        for first_drone, second_drone in bundle.data:
            first_paths = solution.drone_paths[first_drone]
            second_paths = solution.drone_paths[second_drone]
            first_config = solution.get_drone_config(solution.drone_config_mapping[first_drone])
            second_config = solution.get_drone_config(solution.drone_config_mapping[second_drone])

            for first_path_index, first_path in enumerate(first_paths):
                for first_point in range(1, len(first_path) - neighborhood.length):
                    new_path = (0,) + first_path[first_point: first_point + neighborhood.length] + (0,)
                    if solution.calculate_total_weight(new_path) > second_config.capacity:
                        continue
                    if solution.calculate_drone_energy_consumption(
                        new_path,
                        config_index=solution.drone_config_mapping[second_drone],
                        arrival_timestamps=solution.calculate_drone_arrival_timestamps(
                            new_path,
                            config_index=solution.drone_config_mapping[first_drone],
                            offset=solution.drone_timespans[second_drone],
                        ),
                    ) > second_config.battery:
                        continue

                for second_path_index, second_path in enumerate(second_paths):
                    for first_point in range(1, len(first_path) - neighborhood.length):
                        for second_location in range(1, len(second_path) - 1):
                            p1 = list(first_path)
                            p2 = list(second_path)
                            p2[second_location:second_location] = p1[first_point: first_point + neighborhood.length]
                            p1[first_point: first_point + neighborhood.length] = []

                            first_arrival_timestamps = solution.calculate_drone_arrival_timestamps(
                                p1,
                                config_index=solution.drone_config_mapping[first_drone],
                                offset=solution.drone_arrival_timestamps[first_drone][first_path_index - 1][-1] if first_path_index > 0 else 0.0,
                            )
                            second_arrival_timestamps = solution.calculate_drone_arrival_timestamps(
                                p2,
                                config_index=solution.drone_config_mapping[second_drone],
                                offset=solution.drone_arrival_timestamps[second_drone][second_path_index - 1][-1] if second_path_index > 0 else 0.0,
                            )

                            if solution.calculate_total_weight(p1) > first_config.capacity or solution.calculate_total_weight(p2) > second_config.capacity:
                                continue

                            if solution.calculate_drone_energy_consumption(p1, config_index=solution.drone_config_mapping[first_drone], arrival_timestamps=first_arrival_timestamps) > first_config.battery:
                                continue

                            if solution.calculate_drone_energy_consumption(p2, config_index=solution.drone_config_mapping[second_drone], arrival_timestamps=second_arrival_timestamps) > second_config.battery:
                                continue

                            _drone_timespans = list(solution.drone_timespans)
                            _drone_timespans[first_drone] += first_arrival_timestamps[-1] - solution.drone_arrival_timestamps[first_drone][first_path_index][-1]
                            _drone_timespans[second_drone] += second_arrival_timestamps[-1] - solution.drone_arrival_timestamps[second_drone][second_path_index][-1]

                            _drone_waiting_times = list(list(w) for w in solution.drone_waiting_times)
                            _drone_waiting_times[first_drone][first_path_index] = solution.calculate_drone_total_waiting_time(p1, arrival_timestamps=first_arrival_timestamps)
                            _drone_waiting_times[second_drone][second_path_index] = solution.calculate_drone_total_waiting_time(p2, arrival_timestamps=second_arrival_timestamps)
                            factory = SolutionFactory(
                                update_drones=((first_drone, first_path_index, tuple(p1)), (second_drone, second_path_index, tuple(p2))),
                                technician_timespans=solution.technician_timespans,
                                technician_waiting_times=solution.technician_waiting_times,
                                drone_timespans=tuple(_drone_timespans),
                                drone_waiting_times=tuple(tuple(w) for w in solution.drone_waiting_times),
                            )
                            factory.add_to_pareto_set(results)

                            pair = (first_path[first_point], second_path[second_location])
                            swaps_mapping[factory] = (min(pair), max(pair))

        return set((r, swaps_mapping[r]) for r in results)
