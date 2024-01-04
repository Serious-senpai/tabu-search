from __future__ import annotations

from typing import Any, Final, List, Sequence, Tuple, TYPE_CHECKING

from ..mixins import SolutionMetricsMixin
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("SolutionFactory",)


class SolutionFactory(SolutionMetricsMixin):
    """A factory for creating `D2DPathSolution`s. This object holds transformation information with the cost of the target solution,
    not the actual solution (which makes it faster for IPC). The underlying mechanism is as follows:

    1. In the main process, `MultiObjectiveNeighborhood.find_best_candidates` delegates the calculation to worker processes. During
    this IPC, the original solution and the neighborhood with extra data (via `IPCBundle`) are pickled and transfered.
    2. In the worker process, calculations are performed and should create a Pareto set of `SolutionFactory` objects. This set
    is then pickled and returned back to the main process. During IPC, only this set is transfered.
    3. The main process combines the Pareto sets from all worker processes and creates a new Pareto set of `SolutionFactory`.
    Then, it creates a Pareto set of `D2DPathSolution` objects from the `SolutionFactory` objects, using the original solution
    as the argument to `SolutionFactory.from_solution`.

    This process ensures that the data during each IPC is minimal.

    Parameters
    -----
    append_drones:
        The list of paths to be appended to each drone. For example, to append (0, 1, 2, 0) to the first drone and (0, 6, 9, 0) to
        the second one, set `append_drones=[(0, (0, 1, 2, 0)), (1, (0, 6, 9, 0))]`.
    update_drones:
        The list of drone paths to update. For example, to update the second path of the first drone to (0, 1, 2, 0), set
        `update_drones=[(0, 1, (0, 1, 2, 0))]`.
    update_technicians:
        The list of technician paths to update. For example, to update the path of the third technician to (0, 1, 2, 0), set
        `update_technicians=[(2, (0, 1, 2, 0))]`.
    drone_timespans:
        The service timespan of each drone. For example, `drone_timespans[0] = 2700.0` means that the first drone returns to the depot
        after completing the last path at t = 2700.0s.
    drone_waiting_times:
        The total waiting time for each path of each drone. For example, `drone_waiting_times[0][1] = 300.0` means that the second path
        of the first drone has the total waiting time of 300.0s.
    technician_timespans:
        The service timespan of each technician. For example, `technician_timespans[0] = 2700.0` means that the first technician returns
        to the depot at t = 2700.0s.
    technician_waiting_times:
        The total waiting time for each technician. For example, `technician_waiting_times[0] = 1200.0` means that the total waiting time
        for the first technician's path is 1200.0s.
    """

    __slots__ = (
        "__append_drones",
        "__update_drones",
        "__update_technicians",
    )
    if TYPE_CHECKING:
        __append_drones: Final[Sequence[Tuple[int, Tuple[int, ...]]]]
        __update_drones: Final[Sequence[Tuple[int, int, Tuple[int, ...]]]]
        __update_technicians: Final[Sequence[Tuple[int, Tuple[int, ...]]]]

    def __init__(
        self,
        *,
        append_drones: Sequence[Tuple[int, Tuple[int, ...]]] = (),
        update_drones: Sequence[Tuple[int, int, Tuple[int, ...]]] = (),
        update_technicians: Sequence[Tuple[int, Tuple[int, ...]]] = (),
        drone_timespans: Tuple[float, ...],
        drone_waiting_times: Tuple[Tuple[float, ...], ...],
        technician_timespans: Tuple[float, ...],
        technician_waiting_times: Tuple[float, ...],
    ) -> None:
        super().__init__(
            drone_timespans=drone_timespans,
            drone_waiting_times=drone_waiting_times,
            technician_timespans=technician_timespans,
            technician_waiting_times=technician_waiting_times,
        )

        self.__append_drones = append_drones
        self.__update_drones = update_drones
        self.__update_technicians = update_technicians

    def add_violation(self, violation: float) -> None:
        self._fine += violation

    def from_solution(self, __s: D2DPathSolution, /) -> D2DPathSolution:
        if len(self.__append_drones) + len(self.__update_drones) > 0:
            _drone_paths = list(list(paths) for paths in __s.drone_paths)
            for drone, new_path in self.__append_drones:
                if new_path != (0, 0):
                    _drone_paths[drone].append(new_path)

            to_remove: List[Tuple[int, int]] = []
            for drone, drone_path_index, new_path in self.__update_drones:
                _drone_paths[drone][drone_path_index] = new_path
                if new_path == (0, 0):
                    to_remove.append((drone_path_index, drone))

            if len(to_remove) > 0:
                _drone_waiting_times = list(list(p) for p in self.drone_waiting_times)

                to_remove.sort(reverse=True)
                for drone_path_index, drone in to_remove:
                    _drone_paths[drone].pop(drone_path_index)
                    _drone_waiting_times[drone].pop(drone_path_index)

                drone_waiting_times = tuple(tuple(p) for p in _drone_waiting_times)

            else:
                drone_waiting_times = self.drone_waiting_times

            drone_paths = tuple(tuple(paths) for paths in _drone_paths)

        else:
            drone_waiting_times = self.drone_waiting_times
            drone_paths = __s.drone_paths

        if len(self.__update_technicians) > 0:
            _technician_paths = list(__s.technician_paths)
            for technician, new_path in self.__update_technicians:
                _technician_paths[technician] = new_path

            technician_paths = tuple(_technician_paths)

        else:
            technician_paths = __s.technician_paths

        cls = type(__s)
        return cls(
            drone_paths=drone_paths,
            technician_paths=technician_paths,
            drone_timespans=self.drone_timespans,
            drone_waiting_times=drone_waiting_times,
            technician_timespans=self.technician_timespans,
            technician_waiting_times=self.technician_waiting_times,
            fine=self._fine,
        )

    def __hash__(self) -> int:
        return hash((self.__append_drones, self.__update_drones, self.__update_technicians))

    def __eq__(self, other: Any, /) -> bool:
        if isinstance(other, SolutionFactory):
            return (
                self.__append_drones == other.__append_drones
                and self.__update_drones == other.__update_drones
                and self.__update_technicians == other.__update_technicians
            )

        return NotImplemented
