from __future__ import annotations

from typing import List, Sequence, Tuple, TYPE_CHECKING

from ..mixins import SolutionMetricsMixin
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("SolutionFactory",)


class SolutionFactory(SolutionMetricsMixin):
    """Factory for initializing a solution from another one"""

    __slots__ = (
        "__append_drones",
        "__update_drones",
        "__update_technicians",
    )
    if TYPE_CHECKING:
        __append_drones: Sequence[Tuple[int, Tuple[int, ...]]]
        __update_drones: Sequence[Tuple[int, int, Tuple[int, ...]]]
        __update_technicians: Sequence[Tuple[int, Tuple[int, ...]]]

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
            assert len(drone_paths) == len(drone_waiting_times)

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
        )

    def __hash__(self) -> int:
        return hash((self.__update_drones, self.__update_technicians))
