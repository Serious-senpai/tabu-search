from __future__ import annotations

import functools
import itertools
from multiprocessing import pool
from typing import Iterable, List, Optional, Tuple, TYPE_CHECKING

from .mixins import D2DNeighborhoodMixin
from .results import OperationResult
from ..config import DroneEnergyConsumptionMode
from ..errors import NeighborhoodException
from ...abc import BaseNeighborhood
from ...bundle import IPCBundle
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("Swap",)


if TYPE_CHECKING:
    _BaseNeighborhood = BaseNeighborhood[D2DPathSolution, Tuple[int, int]]
else:
    _BaseNeighborhood = BaseNeighborhood


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

        if second_length == 0:
            message = f"Invalid Swap operation: {(first_length, second_length)}"
            raise NeighborhoodException(message)

        self._first_length = first_length
        self._second_length = second_length

    def find_best_candidate(self, *, pool: pool.Pool, pool_size: int) -> Optional[D2DPathSolution]:
        solution = self._solution

        bundles: List[IPCBundle[Swap, List[Tuple[Tuple[int, int], Tuple[int, int]]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
        bundle_iter = itertools.cycle(bundles)

        # Swap between drone paths
        paths: List[Tuple[int, int]] = []
        for index, path in enumerate(solution.drone_paths):
            paths.extend([(index, e) for e in range(len(path))])

        pairs: Iterable[Tuple[Tuple[int, int], Tuple[int, int]]] = itertools.combinations(paths, 2)
        if self._first_length != self._second_length:
            pairs = itertools.permutations(paths, 2)  # type: ignore

        for pair in pairs:
            next(bundle_iter).data.append(pair)

        result: Optional[OperationResult] = None
        min_swap: Optional[Tuple[int, int]] = None
        for result_temp, min_swap_temp in pool.imap(self.swap_drones_paths, bundles):
            if result_temp is None or min_swap_temp is None:
                continue

            if result is None or result_temp < result:
                result = result_temp
                min_swap = min_swap_temp

        if min_swap is not None:
            self.add_to_tabu(min_swap)

        if result is not None:
            return result.to_solution()

        return None

    @staticmethod
    def swap_drones_paths(bundle: IPCBundle[Swap, List[Tuple[Tuple[int, int], Tuple[int, int]]]]) -> Tuple[Optional[OperationResult], Optional[Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood._ensure_imported_data()

        solution = neighborhood._solution
        first_length = neighborhood._first_length
        second_length = neighborhood._second_length
        config = solution.drone_linear_config if solution.energy_mode == DroneEnergyConsumptionMode.LINEAR else solution.drone_nonlinear_config

        drone_paths = list(list(list(path) for path in paths) for paths in solution.drone_paths)
        drone_timespans = list(solution.drone_timespans)
        drone_waiting_times = list(list(w) for w in solution.drone_waiting_times)

        def factory(
            first: Tuple[int, int],
            second: Tuple[int, int],
            first_path: List[int],
            second_path: List[int],
        ) -> D2DPathSolution:
            drone_paths[first[0]][first[1]] = first_path
            drone_paths[second[0]][second[1]] = second_path
            return neighborhood.cls(drone_paths=drone_paths, technician_paths=solution.technician_paths)

        result: Optional[OperationResult] = None
        min_swap: Optional[Tuple[int, int]] = None
        for first, second in bundle.data:
            # Indices of the 2 drones whose paths are altered (can be of the same drone)
            first_drone, first_path_index = first
            second_drone, second_path_index = second

            # The altered paths
            first_path = drone_paths[first_drone][first_path_index]
            second_path = drone_paths[second_drone][second_path_index]

            for first_start, second_start in itertools.product(
                range(1, len(first_path) - first_length),
                range(1, len(second_path) - second_length),
            ):
                _first_path = first_path.copy()
                _second_path = second_path.copy()

                _first_path[first_start:first_start + first_length] = second_path[second_start:second_start + second_length]
                _second_path[second_start:second_start + second_length] = first_path[first_start:first_start + first_length]

                _first_arrival_timestamps = solution.calculate_drone_arrival_timestamps(_first_path, drone=first_drone, offset=0.0)
                _second_arrival_timestamps = solution.calculate_drone_arrival_timestamps(_second_path, drone=second_drone, offset=0.0)
                if solution.calculate_total_weight(_first_path) > config[first_drone].capacity or solution.calculate_total_weight(_second_path) > config[second_drone].capacity:
                    continue

                if solution.calculate_drone_flight_duration(_first_path, drone=first_drone, arrival_timestamps=_first_arrival_timestamps) > solution.drones_flight_duration:
                    continue

                if solution.calculate_drone_flight_duration(_second_path, drone=second_drone, arrival_timestamps=_second_arrival_timestamps) > solution.drones_flight_duration:
                    continue

                if solution.calculate_drone_energy_consumption(_first_path, drone=first_drone, arrival_timestamps=_first_arrival_timestamps) > config[first_drone].battery:
                    continue

                if solution.calculate_drone_energy_consumption(_second_path, drone=second_drone, arrival_timestamps=_second_arrival_timestamps) > config[second_drone].battery:
                    continue

                _drone_timespans = drone_timespans.copy()
                _drone_timespans[first_drone] += _first_arrival_timestamps[-1] - solution.drone_arrival_timestamps[first_drone][first_path_index][-1]
                _drone_timespans[second_drone] += _second_arrival_timestamps[-1] - solution.drone_arrival_timestamps[second_drone][second_path_index][-1]

                _drone_waiting_times = drone_waiting_times.copy()
                _drone_waiting_times[first_drone] = _drone_waiting_times[first_drone].copy()
                _drone_waiting_times[first_drone][first_path_index] = solution.calculate_drone_total_waiting_time(_first_path, drone=first_drone, arrival_timestamps=_first_arrival_timestamps)
                _drone_waiting_times[second_drone] = _drone_waiting_times[second_drone].copy()
                _drone_waiting_times[second_drone][second_path_index] = solution.calculate_drone_total_waiting_time(_second_path, drone=second_drone, arrival_timestamps=_second_arrival_timestamps)

                operation_result = OperationResult(
                    factory=functools.partial(factory, first, second, _first_path, _second_path),
                    drone_timespans=tuple(_drone_timespans),
                    drone_waiting_times=tuple(tuple(p) for p in _drone_waiting_times),
                    technician_timespans=solution.technician_timespans,
                    technician_waiting_times=solution.technician_waiting_times,
                )

                if result is None or operation_result < result:
                    result = operation_result
                    min_swap = (first_path[first_start], second_path[second_start])

        return result, min_swap
