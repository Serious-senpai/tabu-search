from __future__ import annotations

import itertools
from multiprocessing import pool as p
from typing import Dict, Iterable, List, Set, Tuple, TYPE_CHECKING

from .base import D2DBaseNeighborhood
from .factory import SolutionFactory
from ..utils import (
    calculate_drone_arrival_timestamps,
    calculate_technician_arrival_timestamps,
    calculate_drone_total_waiting_time,
    calculate_technician_total_waiting_time,
)
from ..config import DroneEnduranceConfig
from ...bundle import IPCBundle
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


class Insert(D2DBaseNeighborhood[Tuple[Tuple[int, int], int]]):

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
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], int]] = {}

        def callback(candidates: Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], int]]]) -> None:
            for result, pair in candidates:
                if result.add_to_pareto_set(results)[0]:
                    swaps_mapping[result] = pair

        def swap_technician_technician() -> None:
            bundles: List[IPCBundle[Insert, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundles_iter = itertools.cycle(bundles)

            for pair in itertools.permutations(range(solution.technicians_count), 2):
                x = next(bundles_iter)
                x.data.append(pair)  # type: ignore

            for candidates in pool.map(self.swap_technician_technician, bundles):
                callback(candidates)

        def swap_drone_technician() -> None:
            bundles: List[IPCBundle[Insert, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundles_iter = itertools.cycle(bundles)

            for pair in itertools.product(range(solution.drones_count), range(solution.technicians_count)):
                x = next(bundles_iter)
                x.data.append(pair)

            for candidates in pool.map(self.swap_drone_technician, bundles):
                callback(candidates)

        def swap_drone_drone() -> None:
            bundles: List[IPCBundle[Insert, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundles_iter = itertools.cycle(bundles)

            for pair in itertools.permutations(range(solution.drones_count), 2):
                x = next(bundles_iter)
                x.data.append(pair)  # type: ignore

            for candidates in pool.map(self.swap_drone_drone, bundles):
                callback(candidates)

        def swap_technician_drone() -> None:
            bundles: List[IPCBundle[Insert, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundles_iter = itertools.cycle(bundles)

            for pair in itertools.product(range(solution.technicians_count), range(solution.drones_count)):
                x = next(bundles_iter)
                x.data.append(pair)

            for candidates in pool.map(self.swap_technician_drone, bundles):
                callback(candidates)

        for func in (
            swap_technician_technician,
            swap_technician_drone,
            swap_drone_technician,
            swap_drone_drone,
        ):
            func()

        for result in results:
            pair = swaps_mapping[result]
            s = result.from_solution(solution)
            if pair in self.tabu_set:
                s.to_propagate = False

            self.add_to_tabu(pair)
            yield s

    @staticmethod
    def swap_technician_technician(bundle: IPCBundle[Insert, List[Tuple[int, int]]]) -> Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], int]] = {}

        for i, j in bundle.data:
            i_path = solution.technician_paths[i]
            j_path = solution.technician_paths[j]

            for point_i in range(1, len(i_path) - neighborhood.length):
                for location_j in range(1, len(j_path) - 1):
                    pi = list(i_path)
                    pj = list(j_path)

                    pj[location_j:location_j] = i_path[point_i:point_i + neighborhood.length]
                    pi[point_i:point_i + neighborhood.length] = []

                    first_arrival_timestamps = calculate_technician_arrival_timestamps(pi)
                    second_arrival_timestamps = calculate_technician_arrival_timestamps(pj)

                    _technician_timespans = list(solution.technician_timespans)
                    _technician_timespans[i] = first_arrival_timestamps[-1]
                    _technician_timespans[j] = second_arrival_timestamps[-1]

                    _technician_total_waiting_times = list(solution.technician_waiting_times)
                    _technician_total_waiting_times[i] = calculate_technician_total_waiting_time(pi, arrival_timestamps=first_arrival_timestamps)
                    _technician_total_waiting_times[j] = calculate_technician_total_waiting_time(pj, arrival_timestamps=second_arrival_timestamps)

                    factory = SolutionFactory(
                        update_technicians=((i, tuple(pi)), (j, tuple(pj))),
                        drone_timespans=solution.drone_timespans,
                        drone_waiting_times=solution.drone_waiting_times,
                        technician_timespans=tuple(_technician_timespans),
                        technician_waiting_times=tuple(_technician_total_waiting_times),
                    )

                    if factory.add_to_pareto_set(results)[0]:
                        swaps_mapping[factory] = ((i_path[point_i], i_path[point_i + neighborhood.length - 1]), j_path[location_j])

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_drone_drone(bundle: IPCBundle[Insert, List[Tuple[int, int]]]) -> Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], int]] = {}

        config = solution.get_drone_config()
        for first_drone, second_drone in bundle.data:
            first_paths = solution.drone_paths[first_drone]
            second_paths = solution.drone_paths[second_drone]

            for first_path_index, first_path in enumerate(first_paths):

                def create_new() -> None:
                    for first_point in range(1, len(first_path) - neighborhood.length):
                        _first_path = list(first_path)
                        _first_path[first_point:first_point + neighborhood.length] = []
                        _second_path = (0,) + first_path[first_point:first_point + neighborhood.length] + (0,)

                        first_arrival_timestamps = calculate_drone_arrival_timestamps(
                            _first_path,
                            config_type=solution.energy_mode_index,
                            offset=solution.drone_arrival_timestamps[first_drone][first_path_index - 1][-1] if first_path_index > 0 else 0.0,
                        )
                        second_arrival_timestamps = calculate_drone_arrival_timestamps(
                            _second_path,
                            config_type=solution.energy_mode_index,
                            offset=solution.drone_timespans[second_drone],
                        )

                        # No need to check for constraints since splitting a drone path into smaller ones
                        # cannot introduce any violations

                        _drone_timespans = list(solution.drone_timespans)
                        _drone_timespans[first_drone] += first_arrival_timestamps[-1] - solution.drone_arrival_timestamps[first_drone][first_path_index][-1]
                        _drone_timespans[second_drone] += second_arrival_timestamps[-1] - second_arrival_timestamps[0]

                        _drone_waiting_times = list(list(w) for w in solution.drone_waiting_times)
                        _drone_waiting_times[first_drone][first_path_index] = calculate_drone_total_waiting_time(
                            _first_path,
                            arrival_timestamps=first_arrival_timestamps,
                        )
                        _drone_waiting_times[second_drone].append(
                            calculate_drone_total_waiting_time(
                                _second_path,
                                arrival_timestamps=second_arrival_timestamps,
                            )
                        )

                        factory = SolutionFactory(
                            append_drones=((second_drone, _second_path),),
                            update_drones=((first_drone, first_path_index, tuple(_first_path)),),
                            technician_timespans=solution.technician_timespans,
                            technician_waiting_times=solution.technician_waiting_times,
                            drone_timespans=tuple(_drone_timespans),
                            drone_waiting_times=tuple(tuple(w) for w in _drone_waiting_times),
                        )
                        violation = (solution.calculate_total_weight(_second_path) - config.capacity) / config.capacity
                        if violation > 0:
                            factory.add_violation(violation)

                        if factory.add_to_pareto_set(results)[0]:
                            swaps_mapping[factory] = ((first_path[first_point], first_path[first_point + neighborhood.length - 1]), 0)

                # Create a new path for second_drone
                if len(first_path) - 2 > neighborhood.length:
                    create_new()

                # Insert into an existing path of second_drone
                for second_path_index, second_path in enumerate(second_paths):
                    for first_point in range(1, len(first_path) - neighborhood.length):
                        for second_location in range(1, len(second_path) - 1):
                            p1 = list(first_path)
                            p2 = list(second_path)

                            p2[second_location:second_location] = p1[first_point: first_point + neighborhood.length]
                            p1[first_point: first_point + neighborhood.length] = []

                            first_arrival_timestamps = calculate_drone_arrival_timestamps(
                                p1,
                                config_type=solution.energy_mode_index,
                                offset=solution.drone_arrival_timestamps[first_drone][first_path_index - 1][-1] if first_path_index > 0 else 0.0,
                            )
                            second_arrival_timestamps = calculate_drone_arrival_timestamps(
                                p2,
                                config_type=solution.energy_mode_index,
                                offset=solution.drone_arrival_timestamps[second_drone][second_path_index - 1][-1] if second_path_index > 0 else 0.0,
                            )

                            _drone_timespans = list(solution.drone_timespans)
                            _drone_timespans[first_drone] += first_arrival_timestamps[-1] - solution.drone_arrival_timestamps[first_drone][first_path_index][-1]
                            _drone_timespans[second_drone] += second_arrival_timestamps[-1] - solution.drone_arrival_timestamps[second_drone][second_path_index][-1]

                            _drone_waiting_times = list(list(w) for w in solution.drone_waiting_times)
                            _drone_waiting_times[first_drone][first_path_index] = calculate_drone_total_waiting_time(p1, arrival_timestamps=first_arrival_timestamps)
                            _drone_waiting_times[second_drone][second_path_index] = calculate_drone_total_waiting_time(p2, arrival_timestamps=second_arrival_timestamps)

                            factory = SolutionFactory(
                                update_drones=((first_drone, first_path_index, tuple(p1)), (second_drone, second_path_index, tuple(p2))),
                                technician_timespans=solution.technician_timespans,
                                technician_waiting_times=solution.technician_waiting_times,
                                drone_timespans=tuple(_drone_timespans),
                                drone_waiting_times=tuple(tuple(w) for w in _drone_waiting_times),
                            )

                            # The first drone path was shortened, hence it will not violate any constraints as long as the initial
                            # path is also a feasible one

                            violation = (solution.calculate_total_weight(p2) - config.capacity) / config.capacity
                            if violation > 0:
                                factory.add_violation(violation)

                            if isinstance(config, DroneEnduranceConfig):
                                violation = (
                                    solution.calculate_drone_flight_duration(arrival_timestamps=second_arrival_timestamps)
                                    - config.fixed_time
                                ) / config.fixed_time
                                if violation > 0:
                                    factory.add_violation(violation)

                                # No need to check for second_config.fixed_distance since the inserted segment also comes from another
                                # drone path

                            else:
                                violation = (
                                    solution.calculate_drone_energy_consumption(p2)
                                    - config.battery
                                ) / config.battery
                                if violation > 0:
                                    factory.add_violation(violation)

                            if factory.add_to_pareto_set(results)[0]:
                                swaps_mapping[factory] = ((first_path[first_point], first_path[first_point + neighborhood.length - 1]), second_path[second_location])

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_drone_technician(bundle: IPCBundle[Insert, List[Tuple[int, int]]]) -> Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], int]] = {}

        for drone, technician in bundle.data:
            tech_path = solution.technician_paths[technician]
            for drone_path_index, drone_path in enumerate(solution.drone_paths[drone]):
                for drone_point in range(1, len(drone_path) - neighborhood.length):
                    for location_tech in range(1, len(tech_path) - 1):
                        _tech_path = list(tech_path)
                        _drone_path = list(drone_path)

                        _tech_path[location_tech:location_tech] = drone_path[drone_point:drone_point + neighborhood.length]
                        _drone_path[drone_point:drone_point + neighborhood.length] = []

                        tech_arrival_timestamps = calculate_technician_arrival_timestamps(_tech_path)
                        drone_arrival_timestamps = calculate_drone_arrival_timestamps(
                            _drone_path,
                            config_type=solution.energy_mode_index,
                            offset=solution.drone_arrival_timestamps[drone][drone_path_index - 1][-1] if drone_path_index > 0 else 0.0,
                        )

                        _technician_timespans = list(solution.technician_timespans)
                        _technician_timespans[technician] = tech_arrival_timestamps[-1]

                        _drone_timespans = list(solution.drone_timespans)
                        _drone_timespans[drone] += drone_arrival_timestamps[-1] - solution.drone_arrival_timestamps[drone][drone_path_index][-1]

                        _technician_total_waiting_times = list(solution.technician_waiting_times)
                        _technician_total_waiting_times[technician] = calculate_technician_total_waiting_time(_tech_path, arrival_timestamps=tech_arrival_timestamps)

                        _drone_total_waiting_times = list(list(w) for w in solution.drone_waiting_times)
                        _drone_total_waiting_times[drone][drone_path_index] = calculate_drone_total_waiting_time(
                            _drone_path,
                            arrival_timestamps=drone_arrival_timestamps,
                        )

                        factory = SolutionFactory(
                            update_technicians=((technician, tuple(_tech_path)),),
                            update_drones=((drone, drone_path_index, tuple(_drone_path)),),
                            technician_timespans=tuple(_technician_timespans),
                            technician_waiting_times=tuple(_technician_total_waiting_times),
                            drone_timespans=tuple(_drone_timespans),
                            drone_waiting_times=tuple(tuple(w) for w in _drone_total_waiting_times),
                        )
                        if factory.add_to_pareto_set(results)[0]:
                            swaps_mapping[factory] = ((drone_path[drone_point], drone_path[drone_point + neighborhood.length - 1]), tech_path[location_tech])

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_technician_drone(bundle: IPCBundle[Insert, List[Tuple[int, int]]]) -> Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], int]] = {}

        for technician, drone in bundle.data:
            tech_path = solution.technician_paths[technician]
            drone_paths = solution.drone_paths[drone]
            drone_config = solution.get_drone_config()

            dronable_prefix_sum = tuple(itertools.accumulate(solution.dronable[index] for index in tech_path))
            for tech_point in range(1, len(tech_path) - neighborhood.length):
                if dronable_prefix_sum[tech_point + neighborhood.length - 1] - dronable_prefix_sum[tech_point - 1] < neighborhood.length:
                    continue

                def create_new() -> None:
                    _tech_path = list(tech_path)
                    _tech_path[tech_point:tech_point + neighborhood.length] = []
                    _drone_path = (0,) + tuple(tech_path[tech_point:tech_point + neighborhood.length]) + (0,)

                    tech_arrival_timestamps = calculate_technician_arrival_timestamps(_tech_path)
                    drone_arrival_timestamps = calculate_drone_arrival_timestamps(
                        _drone_path,
                        config_type=solution.energy_mode_index,
                        offset=solution.drone_timespans[drone],
                    )

                    _technician_timespans = list(solution.technician_timespans)
                    _technician_timespans[technician] = tech_arrival_timestamps[-1]

                    _drone_timespans = list(solution.drone_timespans)
                    _drone_timespans[drone] += drone_arrival_timestamps[-1] - drone_arrival_timestamps[0]

                    _technician_total_waiting_times = list(solution.technician_waiting_times)
                    _technician_total_waiting_times[technician] = calculate_technician_total_waiting_time(_tech_path, arrival_timestamps=tech_arrival_timestamps)

                    _drone_total_waiting_times = list(list(w) for w in solution.drone_waiting_times)
                    _drone_total_waiting_times[drone].append(
                        calculate_drone_total_waiting_time(
                            _drone_path,
                            arrival_timestamps=drone_arrival_timestamps,
                        )
                    )

                    factory = SolutionFactory(
                        append_drones=((drone, _drone_path),),
                        update_technicians=((technician, tuple(_tech_path)),),
                        technician_timespans=tuple(_technician_timespans),
                        technician_waiting_times=tuple(_technician_total_waiting_times),
                        drone_timespans=tuple(_drone_timespans),
                        drone_waiting_times=tuple(tuple(w) for w in _drone_total_waiting_times),
                    )

                    violation = (solution.calculate_total_weight(_drone_path) - drone_config.capacity) / drone_config.capacity
                    if violation > 0:
                        factory.add_violation(violation)

                    if isinstance(drone_config, DroneEnduranceConfig):
                        violation = (
                            solution.calculate_drone_flight_duration(arrival_timestamps=drone_arrival_timestamps)
                            - drone_config.fixed_time
                        ) / drone_config.fixed_time
                        if violation > 0:
                            factory.add_violation(violation)

                        violation = (solution.calculate_required_range(_drone_path) - drone_config.fixed_distance) / drone_config.fixed_distance
                        if violation > 0:
                            factory.add_violation(violation)

                    else:
                        violation = (
                            solution.calculate_drone_energy_consumption(_drone_path) - drone_config.battery
                        ) / drone_config.battery
                        if violation > 0:
                            factory.add_violation(violation)

                    if factory.add_to_pareto_set(results)[0]:
                        swaps_mapping[factory] = ((tech_path[tech_point], tech_path[tech_point + neighborhood.length - 1]), 0)

                # Create a new drone path
                create_new()

                # Insert into an existing drone path
                for drone_path_index, drone_path in enumerate(drone_paths):
                    for drone_location in range(1, len(drone_path) - 1):
                        _tech_path = list(tech_path)
                        _drone_path = list(drone_path)

                        _tech_path[tech_point:tech_point + neighborhood.length] = []
                        _drone_path[drone_location:drone_location] = tech_path[tech_point:tech_point + neighborhood.length]

                        tech_arrival_timestamps = calculate_technician_arrival_timestamps(_tech_path)
                        drone_arrival_timestamps = calculate_drone_arrival_timestamps(
                            _drone_path,
                            config_type=solution.energy_mode_index,
                            offset=solution.drone_arrival_timestamps[drone][drone_path_index - 1][-1] if drone_path_index > 0 else 0.0,
                        )

                        _technician_timespans = list(solution.technician_timespans)
                        _technician_timespans[technician] = tech_arrival_timestamps[-1]

                        _drone_timespans = list(solution.drone_timespans)
                        _drone_timespans[drone] += drone_arrival_timestamps[-1] - solution.drone_arrival_timestamps[drone][drone_path_index][-1]

                        _technician_total_waiting_times = list(solution.technician_waiting_times)
                        _technician_total_waiting_times[technician] = calculate_technician_total_waiting_time(_tech_path, arrival_timestamps=tech_arrival_timestamps)

                        _drone_total_waiting_times = list(list(w) for w in solution.drone_waiting_times)
                        _drone_total_waiting_times[drone][drone_path_index] = calculate_drone_total_waiting_time(
                            _drone_path,
                            arrival_timestamps=drone_arrival_timestamps,
                        )

                        factory = SolutionFactory(
                            update_technicians=((technician, tuple(_tech_path)),),
                            update_drones=((drone, drone_path_index, tuple(_drone_path)),),
                            technician_timespans=tuple(_technician_timespans),
                            technician_waiting_times=tuple(_technician_total_waiting_times),
                            drone_timespans=tuple(_drone_timespans),
                            drone_waiting_times=tuple(tuple(w) for w in _drone_total_waiting_times),
                        )

                        violation = (solution.calculate_total_weight(_drone_path) - drone_config.capacity) / drone_config.capacity
                        if violation > 0:
                            factory.add_violation(violation)

                        if isinstance(drone_config, DroneEnduranceConfig):
                            violation = (
                                solution.calculate_drone_flight_duration(arrival_timestamps=drone_arrival_timestamps)
                                - drone_config.fixed_time
                            ) / drone_config.fixed_time
                            if violation > 0:
                                factory.add_violation(violation)

                            violation = (solution.calculate_required_range(_drone_path) - drone_config.fixed_distance) / drone_config.fixed_distance
                            if violation > 0:
                                factory.add_violation(violation)

                        else:
                            violation = (
                                solution.calculate_drone_energy_consumption(_drone_path)
                                - drone_config.battery
                            ) / drone_config.battery
                            if violation > 0:
                                factory.add_violation(violation)

                        if factory.add_to_pareto_set(results)[0]:
                            swaps_mapping[factory] = ((tech_path[tech_point], tech_path[tech_point + neighborhood.length - 1]), drone_path[drone_location])

        return set((r, swaps_mapping[r]) for r in results)
