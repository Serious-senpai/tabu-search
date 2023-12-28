from __future__ import annotations

import itertools
from multiprocessing import pool as p
from typing import Dict, Final, Iterable, List, Set, Tuple, TYPE_CHECKING

from .base import D2DBaseNeighborhood
from .factory import SolutionFactory
from ..utils import (
    calculate_drone_arrival_timestamps,
    calculate_technician_arrival_timestamps,
    calculate_drone_total_waiting_time,
    calculate_technician_total_waiting_time,
)
from ..config import DroneEnduranceConfig
from ..errors import NeighborhoodException
from ...bundle import IPCBundle
from ...utils import synchronized
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("Swap",)


class Swap(D2DBaseNeighborhood[Tuple[Tuple[int, int], Tuple[int, int]]]):

    __slots__ = (
        "_first_length",
        "_second_length",
    )
    if TYPE_CHECKING:
        _first_length: Final[int]
        _second_length: Final[int]

    def __init__(self, solution: D2DPathSolution, *, first_length: int, second_length: int) -> None:
        super().__init__(solution)
        if first_length < second_length:
            first_length, second_length = second_length, first_length

        if second_length == 0:
            message = f"Invalid Swap operation: {(first_length, second_length)}"
            raise NeighborhoodException(message)

        self._first_length = first_length
        self._second_length = second_length

    def find_best_candidates(self, *, pool: p.Pool, pool_size: int) -> Iterable[D2DPathSolution]:
        solution = self._solution
        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]] = {}

        @synchronized
        def callback(collected: List[Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]]]]) -> None:
            for s in collected:
                for result, pair in s:
                    if result.add_to_pareto_set(results)[0]:
                        swaps_mapping[result] = pair

        def drone_drone_swap() -> p.MapResult[Set[Tuple[SolutionFactory, Tuple[int, int]]]]:
            bundles: List[IPCBundle[Swap, List[Tuple[Tuple[int, int], Tuple[int, int]]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundle_iter = itertools.cycle(bundles)

            paths: List[Tuple[int, int]] = []
            for drone, path in enumerate(solution.drone_paths):
                paths.extend([(drone, e) for e in range(len(path))])

            pairs: Iterable[Tuple[Tuple[int, int], Tuple[int, int]]] = itertools.combinations(paths, 2)
            if self._first_length != self._second_length:
                pairs = itertools.permutations(paths, 2)  # type: ignore

            drone_paths = solution.drone_paths
            for pair in pairs:
                (first_drone, first_path_index), (second_drone, second_path_index) = pair
                if (
                    len(drone_paths[first_drone][first_path_index]) - 2 > self._first_length
                    or len(drone_paths[second_drone][second_path_index]) - 2 > self._second_length
                ):
                    next(bundle_iter).data.append(pair)

            return pool.map_async(self.swap_drone_drone, bundles, callback=callback)  # type: ignore  # typing bug in multiprocessing.pool module

        def technician_technician_swap() -> p.MapResult[Set[Tuple[SolutionFactory, Tuple[int, int]]]]:
            bundles: List[IPCBundle[Swap, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundle_iter = itertools.cycle(bundles)

            pairs: Iterable[Tuple[int, int]] = itertools.combinations(range(solution.technicians_count), 2)
            if self._first_length != self._second_length:
                pairs = itertools.permutations(range(solution.technicians_count), 2)  # type: ignore

            for pair in pairs:
                next(bundle_iter).data.append(pair)

            # typing bug in multiprocessing.pool module
            return pool.map_async(self.swap_technician_technician, bundles, callback=callback)  # type: ignore  # typing bug in multiprocessing.pool module

        def technician_drone_swap() -> p.MapResult[Set[Tuple[SolutionFactory, Tuple[int, int]]]]:
            bundles: List[IPCBundle[Swap, List[Tuple[int, Tuple[int, int]]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundle_iter = itertools.cycle(bundles)

            drone_paths: List[Tuple[int, int]] = []
            for drone, path in enumerate(solution.drone_paths):
                drone_paths.extend([(drone, e) for e in range(len(path))])

            for arg in itertools.product(range(solution.technicians_count), drone_paths):
                next(bundle_iter).data.append(arg)

            # typing bug in multiprocessing.pool module
            return pool.map_async(self.swap_technician_drone, bundles, callback=callback)  # type: ignore  # typing bug in multiprocessing.pool module

        def drone_self_swap() -> p.MapResult[Set[Tuple[SolutionFactory, Tuple[int, int]]]]:
            bundles: List[IPCBundle[Swap, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundle_iter = itertools.cycle(bundles)

            for drone, paths in enumerate(solution.drone_paths):
                for path_index, path in enumerate(paths):
                    if len(path) - 2 >= self._first_length + self._second_length:
                        next(bundle_iter).data.append((drone, path_index))

            return pool.map_async(self.swap_drone_self, bundles, callback=callback)  # type: ignore  # typing bug in multiprocessing.pool module

        def technician_self_swap() -> p.MapResult[Set[Tuple[SolutionFactory, Tuple[int, int]]]]:
            bundles: List[IPCBundle[Swap, List[int]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundle_iter = itertools.cycle(bundles)

            for technician, path in enumerate(solution.technician_paths):
                if len(path) - 2 >= self._first_length + self._second_length:
                    next(bundle_iter).data.append(technician)

            return pool.map_async(self.swap_technician_self, bundles, callback=callback)  # type: ignore  # typing bug in multiprocessing.pool module

        # Wait for https://github.com/microsoft/pyright/pull/6323 to be merged

        for r in (
            drone_drone_swap(),
            technician_technician_swap(),
            technician_drone_swap(),
            drone_self_swap(),
            technician_self_swap(),
        ):
            r.wait()

        for result in results:
            pair = swaps_mapping[result]
            s = result.from_solution(solution)
            if pair in self.tabu_set:
                s.to_propagate = False

            self.add_to_tabu(pair)

            yield s

    @staticmethod
    def swap_drone_drone(bundle: IPCBundle[Swap, List[Tuple[Tuple[int, int], Tuple[int, int]]]]) -> Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        first_length = neighborhood._first_length
        second_length = neighborhood._second_length

        drone_paths = solution.drone_paths
        drone_timespans = solution.drone_timespans
        drone_waiting_times = solution.drone_waiting_times

        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]] = {}
        for first, second in bundle.data:
            # Indices of the 2 drones whose paths are altered (cannot be of the same drone)
            first_drone, first_path_index = first
            second_drone, second_path_index = second

            # The swapped paths
            first_path = drone_paths[first_drone][first_path_index]
            second_path = drone_paths[second_drone][second_path_index]

            config = solution.get_drone_config()

            for first_start, second_start in itertools.product(
                range(1, len(first_path) - first_length),
                range(1, len(second_path) - second_length),
            ):
                _first_path = list(first_path)
                _second_path = list(second_path)

                _first_path[first_start:first_start + first_length] = second_path[second_start:second_start + second_length]
                _second_path[second_start:second_start + second_length] = first_path[first_start:first_start + first_length]

                first_arrival_timestamps = calculate_drone_arrival_timestamps(
                    _first_path,
                    config_type=solution.energy_mode_index,
                    offset=solution.drone_arrival_timestamps[first_drone][first_path_index - 1][-1] if first_path_index > 0 else 0.0,
                )
                second_arrival_timestamps = calculate_drone_arrival_timestamps(
                    _second_path,
                    config_type=solution.energy_mode_index,
                    offset=solution.drone_arrival_timestamps[second_drone][second_path_index - 1][-1] if second_path_index > 0 else 0.0,
                )

                _drone_timespans = list(drone_timespans)
                _drone_timespans[first_drone] += first_arrival_timestamps[-1] - solution.drone_arrival_timestamps[first_drone][first_path_index][-1]
                _drone_timespans[second_drone] += second_arrival_timestamps[-1] - solution.drone_arrival_timestamps[second_drone][second_path_index][-1]

                _drone_waiting_times = list(list(p) for p in drone_waiting_times)
                _drone_waiting_times[first_drone][first_path_index] = calculate_drone_total_waiting_time(_first_path, arrival_timestamps=first_arrival_timestamps)
                _drone_waiting_times[second_drone][second_path_index] = calculate_drone_total_waiting_time(_second_path, arrival_timestamps=second_arrival_timestamps)

                factory = SolutionFactory(
                    update_drones=((first_drone, first_path_index, tuple(_first_path)), (second_drone, second_path_index, tuple(_second_path))),
                    drone_timespans=tuple(_drone_timespans),
                    drone_waiting_times=tuple(tuple(p) for p in _drone_waiting_times),
                    technician_timespans=solution.technician_timespans,
                    technician_waiting_times=solution.technician_waiting_times,
                )

                violation = (
                    solution.calculate_total_weight(_first_path)
                    - config.capacity
                ) / config.capacity
                if violation > 0:
                    factory.add_violation(violation)

                violation = (
                    solution.calculate_total_weight(_second_path)
                    - config.capacity
                ) / config.capacity
                if violation > 0:
                    factory.add_violation(violation)

                if isinstance(config, DroneEnduranceConfig):
                    violation = (
                        solution.calculate_drone_flight_duration(arrival_timestamps=first_arrival_timestamps)
                        - config.fixed_time
                    ) / config.fixed_time
                    if violation > 0:
                        factory.add_violation(violation)

                    violation = (
                        solution.calculate_required_range(_first_path)
                        - config.fixed_distance
                    ) / config.fixed_distance
                    if violation > 0:
                        factory.add_violation(violation)

                    violation = (
                        solution.calculate_drone_flight_duration(arrival_timestamps=second_arrival_timestamps)
                        - config.fixed_time
                    ) / config.fixed_time
                    if violation > 0:
                        factory.add_violation(violation)

                    violation = (solution.calculate_required_range(_second_path) - config.fixed_distance) / config.fixed_distance
                    if violation > 0:
                        factory.add_violation(violation)

                else:
                    violation = (
                        solution.calculate_drone_energy_consumption(_first_path)
                        - config.battery
                    ) / config.battery
                    if violation > 0:
                        factory.add_violation(violation)

                    violation = (
                        solution.calculate_drone_energy_consumption(_second_path)
                        - config.battery
                    ) / config.battery
                    if violation > 0:
                        factory.add_violation(violation)

                if factory.add_to_pareto_set(results)[0]:
                    swaps_mapping[factory] = (
                        (first_path[first_start], first_path[first_start + first_length - 1]),
                        (second_path[second_start], second_path[second_start + second_length - 1]),
                    )

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_technician_technician(bundle: IPCBundle[Swap, List[Tuple[int, int]]]) -> Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        first_length = neighborhood._first_length
        second_length = neighborhood._second_length

        # Don't alter the variables without a prefix underscore, edit their copies instead
        technician_paths = list(list(path) for path in solution.technician_paths)

        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]] = {}
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

                first_arrival_timestamps = calculate_technician_arrival_timestamps(_first_path)
                second_arrival_timestamps = calculate_technician_arrival_timestamps(_second_path)

                _technician_timespans = list(solution.technician_timespans)
                _technician_timespans[first] = first_arrival_timestamps[-1]
                _technician_timespans[second] = second_arrival_timestamps[-1]

                _technician_total_waiting_times = list(solution.technician_waiting_times)
                _technician_total_waiting_times[first] = calculate_technician_total_waiting_time(_first_path, arrival_timestamps=first_arrival_timestamps)
                _technician_total_waiting_times[second] = calculate_technician_total_waiting_time(_second_path, arrival_timestamps=second_arrival_timestamps)

                factory = SolutionFactory(
                    update_technicians=((first, tuple(_first_path)), (second, tuple(_second_path))),
                    drone_timespans=solution.drone_timespans,
                    drone_waiting_times=solution.drone_waiting_times,
                    technician_timespans=tuple(_technician_timespans),
                    technician_waiting_times=tuple(_technician_total_waiting_times),
                )

                if factory.add_to_pareto_set(results)[0]:
                    swaps_mapping[factory] = (
                        (first_path[first_start], first_path[first_start + first_length - 1]),
                        (second_path[second_start], second_path[second_start + second_length - 1]),
                    )

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_technician_drone(bundle: IPCBundle[Swap, List[Tuple[int, Tuple[int, int]]]]) -> Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution

        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]] = {}

        def populate_results(technician: int, drone: int, drone_path_index: int, technician_length: int, drone_length: int) -> None:
            # Don't alter the variables without a prefix underscore, edit their copies instead
            technician_path = solution.technician_paths[technician]
            drone_path = solution.drone_paths[drone][drone_path_index]
            drone_config = solution.get_drone_config()

            dronable_prefix_sum = tuple(itertools.accumulate(solution.dronable[index] for index in technician_path))
            for technician_start in range(1, len(technician_path) - technician_length):
                if dronable_prefix_sum[technician_start + technician_length - 1] - dronable_prefix_sum[technician_start - 1] == technician_length:
                    # Dronable segment in technician path found
                    for drone_start in range(1, len(drone_path) - drone_length):
                        _technician_path = list(technician_path)
                        _drone_path = list(drone_path)

                        _technician_path[technician_start:technician_start + technician_length] = drone_path[drone_start:drone_start + drone_length]
                        _drone_path[drone_start:drone_start + drone_length] = technician_path[technician_start:technician_start + technician_length]

                        technician_arrival_timestamps = calculate_technician_arrival_timestamps(_technician_path)
                        drone_arrival_timestamps = calculate_drone_arrival_timestamps(
                            _drone_path,
                            config_type=solution.energy_mode_index,
                            offset=solution.drone_arrival_timestamps[drone][drone_path_index - 1][-1] if drone_path_index > 0 else 0.0,
                        )

                        _technician_timespans = list(solution.technician_timespans)
                        _technician_timespans[technician] = technician_arrival_timestamps[-1]
                        _drone_timespans = list(solution.drone_timespans)
                        _drone_timespans[drone] += drone_arrival_timestamps[-1] - solution.drone_arrival_timestamps[drone][drone_path_index][-1]

                        _technician_waiting_times = list(solution.technician_waiting_times)
                        _technician_waiting_times[technician] = calculate_technician_total_waiting_time(_technician_path, arrival_timestamps=technician_arrival_timestamps)
                        _drone_waiting_times = list(list(paths) for paths in solution.drone_waiting_times)
                        _drone_waiting_times[drone][drone_path_index] = calculate_drone_total_waiting_time(_drone_path, arrival_timestamps=drone_arrival_timestamps)

                        factory = SolutionFactory(
                            update_drones=((drone, drone_path_index, tuple(_drone_path)),),
                            update_technicians=((technician, tuple(_technician_path)),),
                            drone_timespans=tuple(_drone_timespans),
                            drone_waiting_times=tuple(tuple(paths) for paths in _drone_waiting_times),
                            technician_timespans=tuple(_technician_timespans),
                            technician_waiting_times=tuple(_technician_waiting_times),
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

                            violation = (
                                solution.calculate_required_range(_drone_path)
                                - drone_config.fixed_distance
                            ) / drone_config.fixed_distance
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
                            swaps_mapping[factory] = (
                                (technician_path[technician_start], technician_path[technician_start + technician_length - 1]),
                                (drone_path[drone_start], drone_path[drone_start + drone_length - 1]),
                            )

        first_length = neighborhood._first_length
        second_length = neighborhood._second_length
        for technician, (drone, drone_path_index) in bundle.data:
            populate_results(technician, drone, drone_path_index, first_length, second_length)
            if first_length != second_length:
                populate_results(technician, drone, drone_path_index, second_length, first_length)

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_drone_self(bundle: IPCBundle[Swap, List[Tuple[int, int]]]) -> Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        first_length = neighborhood._first_length
        second_length = neighborhood._second_length

        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]] = {}

        for drone, path_index in bundle.data:
            path = solution.drone_paths[drone][path_index]
            config = solution.get_drone_config()
            offset = solution.drone_arrival_timestamps[drone][path_index][0]
            for first_index in range(1, len(path) - (first_length + second_length)):
                for second_index in range(first_index + first_length, len(path) - second_length):
                    _path = list(path)

                    # MUST replace [second_index:second_index + second_length] first, otherwise the indices will be incorrect
                    # in case of asymmetric swaps
                    _path[second_index:second_index + second_length] = path[first_index:first_index + first_length]
                    _path[first_index:first_index + first_length] = path[second_index:second_index + second_length]

                    arrival_timestamps = calculate_drone_arrival_timestamps(
                        _path,
                        config_type=solution.energy_mode_index,
                        offset=offset,
                    )

                    _drone_timespans = list(solution.drone_timespans)
                    _drone_timespans[drone] += arrival_timestamps[-1] - solution.drone_arrival_timestamps[drone][path_index][-1]

                    _drone_waiting_times = list(list(p) for p in solution.drone_waiting_times)
                    _drone_waiting_times[drone][path_index] = calculate_drone_total_waiting_time(_path, arrival_timestamps=arrival_timestamps)

                    _drone_paths = list(list(p) for p in solution.drone_paths)
                    _drone_paths[drone][path_index] = tuple(_path)

                    factory = SolutionFactory(
                        update_drones=((drone, path_index, tuple(_path)),),
                        drone_timespans=tuple(_drone_timespans),
                        drone_waiting_times=tuple(tuple(p) for p in _drone_waiting_times),
                        technician_timespans=solution.technician_timespans,
                        technician_waiting_times=solution.technician_waiting_times,
                    )

                    violation = (solution.calculate_total_weight(_path) - config.capacity) / config.capacity
                    if violation > 0:
                        factory.add_violation(violation)

                    if isinstance(config, DroneEnduranceConfig):
                        violation = (
                            solution.calculate_drone_flight_duration(arrival_timestamps=arrival_timestamps)
                            - config.fixed_time
                        ) / config.fixed_time
                        if violation > 0:
                            factory.add_violation(violation)

                        violation = (
                            solution.calculate_required_range(_path)
                            - config.fixed_distance
                        ) / config.fixed_distance
                        if violation > 0:
                            factory.add_violation(violation)

                    else:
                        violation = (
                            solution.calculate_drone_energy_consumption(_path)
                            - config.battery
                        ) / config.battery
                        if violation > 0:
                            factory.add_violation(violation)

                    if factory.add_to_pareto_set(results)[0]:
                        swaps_mapping[factory] = (
                            (path[first_index], path[first_index + first_length - 1]),
                            (path[second_index], path[second_index + second_length - 1]),
                        )

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_technician_self(bundle: IPCBundle[Swap, List[int]]) -> Set[Tuple[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        first_length = neighborhood._first_length
        second_length = neighborhood._second_length

        results: Set[SolutionFactory] = set()
        swaps_mapping: Dict[SolutionFactory, Tuple[Tuple[int, int], Tuple[int, int]]] = {}

        for technician in bundle.data:
            path = solution.technician_paths[technician]
            for first_index in range(1, len(path) - (first_length + second_length)):
                for second_index in range(first_index + first_length, len(path) - second_length):
                    _path = list(path)

                    # MUST replace [second_index:second_index + second_length] first, otherwise the indices will be incorrect
                    # in case of asymmetric swaps
                    _path[second_index:second_index + second_length] = path[first_index:first_index + first_length]
                    _path[first_index:first_index + first_length] = path[second_index:second_index + second_length]

                    arrival_timestamps = calculate_technician_arrival_timestamps(_path)

                    _technician_timespans = list(solution.technician_timespans)
                    _technician_timespans[technician] = arrival_timestamps[-1]

                    _technician_waiting_times = list(solution.technician_waiting_times)
                    _technician_waiting_times[technician] = calculate_technician_total_waiting_time(_path, arrival_timestamps=arrival_timestamps)

                    factory = SolutionFactory(
                        update_technicians=((technician, tuple(_path)),),
                        drone_timespans=solution.drone_timespans,
                        drone_waiting_times=solution.drone_waiting_times,
                        technician_timespans=tuple(_technician_timespans),
                        technician_waiting_times=tuple(_technician_waiting_times),
                    )

                    if factory.add_to_pareto_set(results)[0]:
                        swaps_mapping[factory] = (
                            (path[first_index], path[first_index + first_length - 1]),
                            (path[second_index], path[second_index + second_length - 1]),
                        )

        return set((r, swaps_mapping[r]) for r in results)
