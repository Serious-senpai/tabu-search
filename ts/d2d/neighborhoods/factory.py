from __future__ import annotations

from typing import Sequence, Tuple, TYPE_CHECKING

from ..mixins import SolutionMetricsMixin
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


__all__ = ("SolutionFactory",)


class SolutionFactory(SolutionMetricsMixin):
    """Factory for initializing a solution from another one"""

    __slots__ = (
        "__update_drones",
        "__update_technicians",
    )
    if TYPE_CHECKING:
        __update_drones: Sequence[Tuple[int, int, Tuple[int, ...]]]
        __update_technicians: Sequence[Tuple[int, Tuple[int, ...]]]

    def __init__(
        self,
        *,
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

        self.__update_drones = update_drones
        self.__update_technicians = update_technicians

    def from_solution(self, __s: D2DPathSolution, /) -> D2DPathSolution:
        if len(self.__update_drones) > 0:
            _drone_paths = list(list(paths) for paths in __s.drone_paths)
            for drone, drone_path_index, new_path in self.__update_drones:
                _drone_paths[drone][drone_path_index] = new_path

            drone_paths = tuple(tuple(paths) for paths in _drone_paths)

        else:
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
            drone_config_mapping=__s.drone_config_mapping,
        )

    def __hash__(self) -> int:
        return hash((self.__update_drones, self.__update_technicians))
