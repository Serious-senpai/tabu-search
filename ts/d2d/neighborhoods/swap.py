from __future__ import annotations

import functools
import itertools
from copy import deepcopy
from multiprocessing import pool as p
from typing import Dict, Iterable, List, Set, Tuple, TYPE_CHECKING

from .mixins import D2DNeighborhoodMixin
from .results import OperationResult
from ..config import DroneEnergyConsumptionMode
from ..errors import NeighborhoodException
from ...abc import MultiObjectiveNeighborhood
from ...bundle import IPCBundle
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("Swap",)


if TYPE_CHECKING:
    _BaseNeighborhood = MultiObjectiveNeighborhood[D2DPathSolution, Tuple[int, int]]
else:
    _BaseNeighborhood = MultiObjectiveNeighborhood


class Swap(D2DNeighborhoodMixin, _BaseNeighborhood):

    __slots__ = (
        "_first_length",
        "_second_length",
    )
    if TYPE_CHECKING:
        _first_length: int
        _second_length: int

    def __init__(self, solution: D2DPathSolution, *, first_length: int, second_length: int) -> None:
        super().__init__(solution)
        if first_length < second_length:
            first_length, second_length = second_length, first_length

        self._first_length = first_length
        self._second_length = second_length

    def find_best_candidates(self, *, pool: p.Pool, pool_size: int) -> Iterable[D2DPathSolution]:
        solution = self._solution
        results: Set[OperationResult] = set()
        swaps_mapping: Dict[OperationResult, Tuple[int, int]] = {}

        def callback(collected: Iterable[Set[Tuple[OperationResult, Tuple[int, int]]]]) -> None:
            for s in collected:
                for result, pair in s:
                    swaps_mapping[result] = pair
                    result.add_to_pareto_set(results)

        def drone_drone_swap() -> p.MapResult[Set[Tuple[OperationResult, Tuple[int, int]]]]:
            bundles: List[IPCBundle[Swap, List[Tuple[Tuple[int, int], Tuple[int, int]]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundle_iter = itertools.cycle(bundles)

            paths: List[Tuple[int, int]] = []
            for drone, path in enumerate(solution.drone_paths):
                paths.extend([(drone, e) for e in range(len(path))])

            pairs: Iterable[Tuple[Tuple[int, int], Tuple[int, int]]] = itertools.combinations(paths, 2)
            if self._first_length != self._second_length:
                pairs = itertools.permutations(paths, 2)  # type: ignore

            for pair in pairs:
                next(bundle_iter).data.append(pair)

            return pool.map_async(self.swap_drone_drone, bundles, callback=callback)  # type: ignore  # typing bug in multiprocessing.pool module

        def technician_technician_swap() -> p.MapResult[Set[Tuple[OperationResult, Tuple[int, int]]]]:
            bundles: List[IPCBundle[Swap, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundle_iter = itertools.cycle(bundles)

            pairs: Iterable[Tuple[int, int]] = itertools.combinations(range(solution.technicians_count), 2)
            if self._first_length != self._second_length:
                pairs = itertools.permutations(range(solution.technicians_count), 2)  # type: ignore

            for pair in pairs:
                next(bundle_iter).data.append(pair)

            # typing bug in multiprocessing.pool module
            return pool.map_async(self.swap_technician_technician, bundles, callback=callback)  # type: ignore  # typing bug in multiprocessing.pool module

        def technician_drone_swap() -> p.MapResult[Set[Tuple[OperationResult, Tuple[int, int]]]]:
            bundles: List[IPCBundle[Swap, List[Tuple[int, Tuple[int, int]]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundle_iter = itertools.cycle(bundles)

            drone_paths: List[Tuple[int, int]] = []
            for drone, path in enumerate(solution.drone_paths):
                drone_paths.extend([(drone, e) for e in range(len(path))])

            for arg in itertools.product(range(solution.technicians_count), drone_paths):
                next(bundle_iter).data.append(arg)

            # typing bug in multiprocessing.pool module
            return pool.map_async(self.swap_technician_drone, bundles, callback=callback)  # type: ignore  # typing bug in multiprocessing.pool module

        def drone_self_swap() -> p.MapResult[Set[Tuple[OperationResult, Tuple[int, int]]]]:
            bundles: List[IPCBundle[Swap, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundle_iter = itertools.cycle(bundles)

            for drone, path in enumerate(solution.drone_paths):
                for path_index in range(len(path)):
                    next(bundle_iter).data.append((drone, path_index))

            return pool.map_async(self.swap_drone_self, bundles, callback=callback)  # type: ignore  # typing bug in multiprocessing.pool module

        def technician_self_swap() -> p.MapResult[Set[Tuple[OperationResult, Tuple[int, int]]]]:
            bundles: List[IPCBundle[Swap, List[int]]] = [IPCBundle(self, []) for _ in range(pool_size)]
            bundle_iter = itertools.cycle(bundles)

            for technician in range(solution.technicians_count):
                next(bundle_iter).data.append(technician)

            return pool.map_async(self.swap_technician_self, bundles, callback=callback)  # type: ignore  # typing bug in multiprocessing.pool module

        # Wait for https://github.com/python/typeshed/pull/10949 to be merged

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
            solution = result.to_solution()
            if pair in self.tabu_set:
                solution.to_propagate = False

            else:
                self.add_to_tabu((min(pair), max(pair)))

            yield solution

    @staticmethod
    def swap_drone_drone(bundle: IPCBundle[Swap, List[Tuple[Tuple[int, int], Tuple[int, int]]]]) -> Set[Tuple[OperationResult, Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        first_length = neighborhood._first_length
        second_length = neighborhood._second_length

        # Don't alter the variables without a prefix underscore, edit their copies instead
        drone_paths = list(list(list(path) for path in paths) for paths in solution.drone_paths)
        drone_timespans = list(solution.drone_timespans)
        drone_waiting_times = list(list(w) for w in solution.drone_waiting_times)

        results: Set[OperationResult] = set()
        swaps_mapping: Dict[OperationResult, Tuple[int, int]] = {}
        for first, second in bundle.data:
            # Indices of the 2 drones whose paths are altered (cannot be of the same drone)
            first_drone, first_path_index = first
            second_drone, second_path_index = second

            # The swapped paths
            first_path = drone_paths[first_drone][first_path_index]
            second_path = drone_paths[second_drone][second_path_index]

            first_config = solution.get_drone_config(solution.drone_config_mapping[first_drone])
            second_config = solution.get_drone_config(solution.drone_config_mapping[second_drone])

            for first_start, second_start in itertools.product(
                range(1, len(first_path) - first_length),
                range(1, len(second_path) - second_length),
            ):
                _first_path = first_path.copy()
                _second_path = second_path.copy()

                _first_path[first_start:first_start + first_length] = second_path[second_start:second_start + second_length]
                _second_path[second_start:second_start + second_length] = first_path[first_start:first_start + first_length]

                first_arrival_timestamps = solution.calculate_drone_arrival_timestamps(
                    _first_path,
                    config_index=solution.drone_config_mapping[first_drone],
                    offset=solution.drone_arrival_timestamps[first_drone][first_path_index - 1][-1] if first_path_index > 0 else 0.0,
                )
                second_arrival_timestamps = solution.calculate_drone_arrival_timestamps(
                    _second_path,
                    config_index=solution.drone_config_mapping[second_drone],
                    offset=solution.drone_arrival_timestamps[second_drone][second_path_index - 1][-1] if second_path_index > 0 else 0.0,
                )
                if solution.calculate_total_weight(_first_path) > first_config.capacity or solution.calculate_total_weight(_second_path) > second_config.capacity:
                    continue

                if solution.calculate_drone_energy_consumption(_first_path, config_index=solution.drone_config_mapping[first_drone], arrival_timestamps=first_arrival_timestamps) > first_config.battery:
                    continue

                if solution.calculate_drone_energy_consumption(_second_path, config_index=solution.drone_config_mapping[second_drone], arrival_timestamps=second_arrival_timestamps) > second_config.battery:
                    continue

                _drone_timespans = drone_timespans.copy()
                _drone_timespans[first_drone] += first_arrival_timestamps[-1] - solution.drone_arrival_timestamps[first_drone][first_path_index][-1]
                _drone_timespans[second_drone] += second_arrival_timestamps[-1] - solution.drone_arrival_timestamps[second_drone][second_path_index][-1]

                _drone_waiting_times = drone_waiting_times.copy()
                _drone_waiting_times[first_drone] = _drone_waiting_times[first_drone].copy()
                _drone_waiting_times[first_drone][first_path_index] = solution.calculate_drone_total_waiting_time(_first_path, config_index=solution.drone_config_mapping[first_drone], arrival_timestamps=first_arrival_timestamps)
                _drone_waiting_times[second_drone] = _drone_waiting_times[second_drone].copy()
                _drone_waiting_times[second_drone][second_path_index] = solution.calculate_drone_total_waiting_time(_second_path, config_index=solution.drone_config_mapping[second_drone], arrival_timestamps=second_arrival_timestamps)

                _drone_paths = deepcopy(drone_paths)
                _drone_paths[first[0]][first[1]] = _first_path
                _drone_paths[second[0]][second[1]] = _second_path

                operation_result = OperationResult(
                    factory=functools.partial(neighborhood.cls, drone_paths=_drone_paths, technician_paths=solution.technician_paths, drone_config_mapping=solution.drone_config_mapping),
                    drone_timespans=tuple(_drone_timespans),
                    drone_waiting_times=tuple(tuple(p) for p in _drone_waiting_times),
                    technician_timespans=solution.technician_timespans,
                    technician_waiting_times=solution.technician_waiting_times,
                )

                pair = (first_path[first_start], second_path[second_start])
                swaps_mapping[operation_result] = (min(pair), max(pair))
                operation_result.add_to_pareto_set(results)

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_technician_technician(bundle: IPCBundle[Swap, List[Tuple[int, int]]]) -> Set[Tuple[OperationResult, Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        first_length = neighborhood._first_length
        second_length = neighborhood._second_length

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
                    factory=functools.partial(neighborhood.cls, drone_paths=solution.drone_paths, technician_paths=_technician_paths, drone_config_mapping=solution.drone_config_mapping),
                    drone_timespans=solution.drone_timespans,
                    drone_waiting_times=solution.drone_waiting_times,
                    technician_timespans=tuple(_technician_timespans),
                    technician_waiting_times=tuple(_technician_total_waiting_times),
                )

                pair = (first_path[first_start], second_path[second_start])
                swaps_mapping[operation_result] = (min(pair), max(pair))
                operation_result.add_to_pareto_set(results)

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_technician_drone(bundle: IPCBundle[Swap, List[Tuple[int, Tuple[int, int]]]]) -> Set[Tuple[OperationResult, Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution

        results: Set[OperationResult] = set()
        swaps_mapping: Dict[OperationResult, Tuple[int, int]] = {}

        def populate_results(technician: int, drone: int, drone_path_index: int, technician_length: int, drone_length: int) -> None:
            # Don't alter the variables without a prefix underscore, edit their copies instead
            technician_path = solution.technician_paths[technician]
            drone_path = solution.drone_paths[drone][drone_path_index]
            drone_config = solution.get_drone_config(solution.drone_config_mapping[drone])

            dronable_prefix_sum = tuple(itertools.accumulate(solution.dronable[index] for index in technician_path))
            for technician_start in range(1, len(technician_path) - technician_length):
                if dronable_prefix_sum[technician_start + technician_length - 1] - dronable_prefix_sum[technician_start - 1] == technician_length:
                    # Dronable segment in technician path found
                    for drone_start in range(1, len(drone_path) - drone_length):
                        _technician_path = list(technician_path)
                        _drone_path = list(drone_path)

                        _technician_path[technician_start:technician_start + technician_length] = drone_path[drone_start:drone_start + drone_length]
                        _drone_path[drone_start:drone_start + drone_length] = technician_path[technician_start:technician_start + technician_length]

                        technician_arrival_timestamps = solution.calculate_technician_arrival_timestamps(_technician_path)
                        drone_arrival_timestamps = solution.calculate_drone_arrival_timestamps(
                            _drone_path,
                            config_index=solution.drone_config_mapping[drone],
                            offset=solution.drone_arrival_timestamps[drone][drone_path_index - 1][-1] if drone_path_index > 0 else 0.0,
                        )

                        if solution.calculate_total_weight(_drone_path) > drone_config.capacity:
                            continue

                        if solution.calculate_drone_energy_consumption(_drone_path, config_index=solution.drone_config_mapping[drone], arrival_timestamps=drone_arrival_timestamps) > drone_config.battery:
                            continue

                        _technician_timespans = list(solution.technician_timespans)
                        _technician_timespans[technician] = technician_arrival_timestamps[-1]
                        _drone_timespans = list(solution.drone_timespans)
                        _drone_timespans[drone] += drone_arrival_timestamps[-1] - solution.drone_arrival_timestamps[drone][drone_path_index][-1]

                        _technician_waiting_times = list(solution.technician_waiting_times)
                        _technician_waiting_times[technician] = solution.calculate_technician_total_waiting_time(_technician_path, arrival_timestamps=technician_arrival_timestamps)
                        _drone_waiting_times = list(list(paths) for paths in solution.drone_waiting_times)
                        _drone_waiting_times[drone][drone_path_index] = solution.calculate_drone_total_waiting_time(_drone_path, config_index=solution.drone_config_mapping[drone], arrival_timestamps=drone_arrival_timestamps)

                        _technician_paths = list(solution.technician_paths)
                        _technician_paths[technician] = tuple(_technician_path)

                        _drone_paths = list(list(paths) for paths in solution.drone_paths)
                        _drone_paths[drone][drone_path_index] = tuple(_drone_path)

                        operation_result = OperationResult(
                            factory=functools.partial(neighborhood.cls, drone_paths=_drone_paths, technician_paths=_technician_paths, drone_config_mapping=solution.drone_config_mapping),
                            drone_timespans=tuple(_drone_timespans),
                            drone_waiting_times=tuple(tuple(paths) for paths in _drone_waiting_times),
                            technician_timespans=tuple(_technician_timespans),
                            technician_waiting_times=tuple(_technician_waiting_times),
                        )

                        pair = (technician_path[technician_start], drone_path[drone_start])
                        swaps_mapping[operation_result] = (min(pair), max(pair))
                        operation_result.add_to_pareto_set(results)

        first_length = neighborhood._first_length
        second_length = neighborhood._second_length
        for technician, (drone, drone_path_index) in bundle.data:
            populate_results(technician, drone, drone_path_index, first_length, second_length)
            if first_length != second_length:
                populate_results(technician, drone, drone_path_index, second_length, first_length)

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_drone_self(bundle: IPCBundle[Swap, List[Tuple[int, int]]]) -> Set[Tuple[OperationResult, Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        first_length = neighborhood._first_length
        second_length = neighborhood._second_length

        results: Set[OperationResult] = set()
        swaps_mapping: Dict[OperationResult, Tuple[int, int]] = {}

        for drone, path_index in bundle.data:
            path = solution.drone_paths[drone][path_index]
            config = solution.get_drone_config(solution.drone_config_mapping[drone])
            offset = solution.drone_arrival_timestamps[drone][path_index][0]
            for first_index in range(1, len(path) - (first_length + second_length)):
                for second_index in range(first_index + first_length, len(path) - second_length):
                    _path = list(path)

                    # MUST replace [second_index:second_index + second_length] first
                    _path[second_index:second_index + second_length] = path[first_index:first_index + first_length]
                    _path[first_index:first_index + first_length] = path[second_index:second_index + second_length]

                    arrival_timestamps = solution.calculate_drone_arrival_timestamps(
                        _path,
                        config_index=solution.drone_config_mapping[drone],
                        offset=offset,
                    )

                    if solution.calculate_total_weight(_path) > config.capacity:
                        continue

                    if solution.calculate_drone_energy_consumption(_path, config_index=solution.drone_config_mapping[drone], arrival_timestamps=arrival_timestamps) > config.battery:
                        continue

                    _drone_timespans = list(solution.drone_timespans)
                    _drone_timespans[drone] += arrival_timestamps[-1] - solution.drone_arrival_timestamps[drone][path_index][-1]

                    _drone_waiting_times = list(list(p) for p in solution.drone_waiting_times)
                    _drone_waiting_times[drone][path_index] = solution.calculate_drone_total_waiting_time(
                        _path,
                        config_index=solution.drone_config_mapping[drone],
                        arrival_timestamps=arrival_timestamps,
                    )

                    _drone_paths = list(list(p) for p in solution.drone_paths)
                    _drone_paths[drone][path_index] = tuple(_path)

                    operation_result = OperationResult(
                        factory=functools.partial(neighborhood.cls, drone_paths=_drone_paths, technician_paths=solution.technician_paths, drone_config_mapping=solution.drone_config_mapping),
                        drone_timespans=tuple(_drone_timespans),
                        drone_waiting_times=tuple(tuple(p) for p in _drone_waiting_times),
                        technician_timespans=solution.technician_timespans,
                        technician_waiting_times=solution.technician_waiting_times,
                    )

                    pair = (path[first_index], path[second_index])
                    swaps_mapping[operation_result] = (min(pair), max(pair))
                    operation_result.add_to_pareto_set(results)

        return set((r, swaps_mapping[r]) for r in results)

    @staticmethod
    def swap_technician_self(bundle: IPCBundle[Swap, List[int]]) -> Set[Tuple[OperationResult, Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        solution = neighborhood._solution
        first_length = neighborhood._first_length
        second_length = neighborhood._second_length

        results: Set[OperationResult] = set()
        swaps_mapping: Dict[OperationResult, Tuple[int, int]] = {}

        for technician in bundle.data:
            path = solution.technician_paths[technician]
            for first_index in range(1, len(path) - (first_length + second_length)):
                for second_index in range(first_index + first_length, len(path) - second_length):
                    _path = list(path)

                    # MUST replace [second_index:second_index + second_length] first
                    _path[second_index:second_index + second_length] = path[first_index:first_index + first_length]
                    _path[first_index:first_index + first_length] = path[second_index:second_index + second_length]

                    arrival_timestamps = solution.calculate_technician_arrival_timestamps(_path)

                    _technician_timespans = list(solution.technician_timespans)
                    _technician_timespans[technician] = arrival_timestamps[-1]

                    _technician_waiting_times = list(solution.technician_waiting_times)
                    _technician_waiting_times[technician] = solution.calculate_technician_total_waiting_time(_path, arrival_timestamps=arrival_timestamps)

                    _technician_paths = list(solution.technician_paths)
                    _technician_paths[technician] = tuple(_path)

                    operation_result = OperationResult(
                        factory=functools.partial(neighborhood.cls, drone_paths=solution.drone_paths, technician_paths=_technician_paths, drone_config_mapping=solution.drone_config_mapping),
                        drone_timespans=solution.drone_timespans,
                        drone_waiting_times=solution.drone_waiting_times,
                        technician_timespans=tuple(_technician_timespans),
                        technician_waiting_times=tuple(_technician_waiting_times),
                    )

                    pair = (path[first_index], path[second_index])
                    swaps_mapping[operation_result] = (min(pair), max(pair))
                    operation_result.add_to_pareto_set(results)

        return set((r, swaps_mapping[r]) for r in results)
