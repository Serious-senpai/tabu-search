from __future__ import annotations

import contextlib
import itertools
import pickle
import random
import re
from math import sqrt
from multiprocessing import shared_memory
from os.path import join
from typing import Any, Callable, ClassVar, Final, List, Literal, Optional, Sequence, Set, Tuple, Union, TYPE_CHECKING, final, overload

from matplotlib import axes, pyplot

from .config import DroneEnduranceConfig, DroneEnergyConsumptionMode, DroneLinearConfig, DroneNonlinearConfig, TruckConfig
from .errors import ImportException
from .mixins import SolutionMetricsMixin
from .neighborhoods import Swap, Insert
from ..abc import MultiObjectiveNeighborhood, MultiObjectiveSolution


__all__ = ("D2DPathSolution",)


@final
class D2DPathSolution(SolutionMetricsMixin, MultiObjectiveSolution):
    """Represents a solution to the D2D problem"""

    __slots__ = (
        "_to_propagate",
        "drone_arrival_timestamps",
        "drone_paths",
        "technician_arrival_timestamps",
        "technician_paths",
    )
    __config_imported: ClassVar[bool] = False
    problem: ClassVar[Optional[str]] = None
    if TYPE_CHECKING:
        _to_propagate: bool
        drone_arrival_timestamps: Final[Tuple[Tuple[Tuple[float, ...], ...], ...]]
        drone_paths: Final[Tuple[Tuple[Tuple[int, ...], ...], ...]]
        technician_arrival_timestamps: Final[Tuple[Tuple[float, ...], ...]]
        technician_paths: Final[Tuple[Tuple[int, ...], ...]]

        # Problem-specific data
        customers_count: ClassVar[int]
        drones_count: ClassVar[int]
        technicians_count: ClassVar[int]

        x: ClassVar[Tuple[float, ...]]
        y: ClassVar[Tuple[float, ...]]
        distances: ClassVar[Tuple[Tuple[float, ...], ...]]
        demands: ClassVar[Tuple[float, ...]]
        dronable: ClassVar[Tuple[bool, ...]]
        drone_service_time: ClassVar[Tuple[float, ...]]
        technician_service_time: ClassVar[Tuple[float, ...]]

        # Global configuration data
        drone_config_mapping: ClassVar[Tuple[int, ...]]
        energy_mode: ClassVar[DroneEnergyConsumptionMode]
        truck_config: ClassVar[TruckConfig]
        drone_linear_config: ClassVar[Tuple[DroneLinearConfig, ...]]
        drone_nonlinear_config: ClassVar[Tuple[DroneNonlinearConfig, ...]]
        drone_endurance_config: ClassVar[Tuple[DroneEnduranceConfig, ...]]

    def __init__(
        self,
        *,
        drone_paths: Tuple[Tuple[Tuple[int, ...], ...], ...],
        technician_paths: Tuple[Tuple[int, ...], ...],
        drone_arrival_timestamps: Optional[Tuple[Tuple[Tuple[float, ...], ...], ...]] = None,
        technician_arrival_timestamps: Optional[Tuple[Tuple[float, ...], ...]] = None,
        drone_timespans: Optional[Tuple[float, ...]] = None,
        drone_waiting_times: Optional[Tuple[Tuple[float, ...], ...]] = None,
        technician_timespans: Optional[Tuple[float, ...]] = None,
        technician_waiting_times: Optional[Tuple[float, ...]] = None,
    ) -> None:
        self._to_propagate = True
        self.drone_paths = drone_paths
        self.technician_paths = technician_paths

        if drone_arrival_timestamps is None:
            def get_arrival_timestamps() -> Tuple[Tuple[Tuple[float, ...], ...], ...]:
                result: List[List[Tuple[float, ...]]] = []
                for drone, paths in enumerate(drone_paths):
                    drone_arrivals: List[Tuple[float, ...]] = []
                    result.append(drone_arrivals)

                    offset = 0.0
                    for path in paths:
                        arrivals = self.calculate_drone_arrival_timestamps(path, config_index=self.drone_config_mapping[drone], offset=offset)
                        offset = arrivals[-1]
                        drone_arrivals.append(arrivals)

                return tuple(tuple(paths) for paths in result)

            drone_arrival_timestamps = get_arrival_timestamps()

        self.drone_arrival_timestamps = drone_arrival_timestamps

        if technician_arrival_timestamps is None:
            technician_arrival_timestamps = tuple(self.calculate_technician_arrival_timestamps(path) for path in technician_paths)

        self.technician_arrival_timestamps = technician_arrival_timestamps

        def __last_element(__tuple: Tuple[Tuple[float, ...], ...]) -> float:
            try:
                return __tuple[-1][-1]
            except IndexError:
                return 0.0

        super().__init__(
            drone_timespans=drone_timespans or tuple(__last_element(single_drone_arrival_timestamps) for single_drone_arrival_timestamps in self.drone_arrival_timestamps),
            drone_waiting_times=drone_waiting_times or tuple(
                tuple(
                    self.calculate_drone_total_waiting_time(path, arrival_timestamps=arrival_timestamps)
                    for path, arrival_timestamps in zip(paths, self.drone_arrival_timestamps[drone])
                )
                for drone, paths in enumerate(drone_paths)
            ),
            technician_timespans=technician_timespans or tuple(technician_arrival_timestamp[-1] for technician_arrival_timestamp in self.technician_arrival_timestamps),
            technician_waiting_times=technician_waiting_times or tuple(
                self.calculate_technician_total_waiting_time(path, arrival_timestamps=arrival_timestamps)
                for path, arrival_timestamps in zip(technician_paths, self.technician_arrival_timestamps)
            ),
        )

    @property
    def to_propagate(self) -> bool:
        return self._to_propagate

    @to_propagate.setter
    def to_propagate(self, propagate: bool) -> None:
        self._to_propagate = propagate

    def shuffle(self, *, use_tqdm: bool, logger: Optional[Callable[[str], Any]]) -> D2DPathSolution:
        drone_paths = list(list(paths) for paths in self.drone_paths)
        technician_paths = list(self.technician_paths)

        for drone, paths in enumerate(drone_paths):
            for path_index, path in enumerate(paths):
                if random.random() < 0.5:
                    drone_paths[drone][path_index] = tuple(reversed(path))

        for technician, path in enumerate(technician_paths):
            if random.random() < 0.5:
                technician_paths[technician] = tuple(reversed(path))

        return D2DPathSolution(
            drone_paths=tuple(tuple(paths) for paths in drone_paths),
            technician_paths=tuple(technician_paths),
            drone_timespans=self.drone_timespans,
            technician_timespans=self.technician_timespans,
        )

    def get_neighborhoods(self) -> Tuple[MultiObjectiveNeighborhood[D2DPathSolution, Any], ...]:
        return (
            Swap(self, first_length=1, second_length=1),
            Swap(self, first_length=2, second_length=1),
            Swap(self, first_length=2, second_length=2),
            Insert(self, length=1),
            Insert(self, length=2),
        )

    def feasible(self) -> bool:
        existed: Set[int] = set()
        for drone, drone_paths in enumerate(self.drone_paths):
            config = self.get_drone_config(self.drone_config_mapping[drone])
            for drone_path_index, drone_path in enumerate(drone_paths):
                if drone_path[0] != 0 or drone_path[-1] != 0:
                    return False

                for index in drone_path[1:-1]:
                    if index in existed:
                        return False

                    existed.add(index)

                if self.calculate_total_weight(drone_path) > config.capacity:
                    return False

                if self.calculate_drone_energy_consumption(
                    drone_path,
                    config_index=self.drone_config_mapping[drone],
                    arrival_timestamps=self.drone_arrival_timestamps[drone][drone_path_index],
                ) > config.battery:
                    return False

        for technician_path in self.technician_paths:
            if technician_path[0] != 0 or technician_path[-1] != 0:
                return False

            for index in technician_path[1:-1]:
                if index in existed:
                    return False

                existed.add(index)

        return len(existed) == self.customers_count

    def plot(self) -> None:
        _, ax = pyplot.subplots()
        assert isinstance(ax, axes.Axes)

        for paths in self.drone_paths:
            drone_x: List[float] = []
            drone_y: List[float] = []
            drone_u: List[float] = []
            drone_v: List[float] = []
            for path in paths:
                for index in range(len(path) - 1):
                    current = path[index]
                    after = path[index + 1]

                    drone_x.append(self.x[current])
                    drone_y.append(self.y[current])
                    drone_u.append(self.x[after] - self.x[current])
                    drone_v.append(self.y[after] - self.y[current])

            ax.quiver(
                drone_x,
                drone_y,
                drone_u,
                drone_v,
                color="cyan",
                angles="xy",
                scale_units="xy",
                scale=1,
            )

        for path in self.technician_paths:
            technician_x: List[float] = []
            technician_y: List[float] = []
            technician_u: List[float] = []
            technician_v: List[float] = []
            for index in range(len(path) - 1):
                current = path[index]
                after = path[index + 1]

                technician_x.append(self.x[current])
                technician_y.append(self.y[current])
                technician_u.append(self.x[after] - self.x[current])
                technician_v.append(self.y[after] - self.y[current])

            ax.quiver(
                technician_x,
                technician_y,
                technician_u,
                technician_v,
                color="darkviolet",
                angles="xy",
                scale_units="xy",
                scale=1,
            )

        ax.scatter((0,), (0,), c="black", label="Deport")
        ax.scatter(
            [self.x[index] for index in range(1, 1 + self.customers_count) if self.dronable[index]],
            [self.y[index] for index in range(1, 1 + self.customers_count) if self.dronable[index]],
            c="darkblue",
            label="Dronable",
        )
        ax.scatter(
            [self.x[index] for index in range(1, 1 + self.customers_count) if not self.dronable[index]],
            [self.y[index] for index in range(1, 1 + self.customers_count) if not self.dronable[index]],
            c="red",
            label="Technician-only",
        )

        ax.annotate("0", (0, 0))
        for index in range(1, 1 + self.customers_count):
            ax.annotate(str(index), (self.x[index], self.y[index]))

        ax.grid(True)

        pyplot.legend()
        pyplot.show()

    @classmethod
    def calculate_drone_arrival_timestamps(cls, path: Sequence[int], *, config_index: int, offset: float) -> Tuple[float, ...]:
        """Calculate the arrival timestamps for the given drone path

        Parameters
        -----
        path:
            The path to calculate
        config_index:
            The index of the drone config to use
        offset:
            The timestamp when the drone starts traveling

        Returns
        -----
        The arrival timestamps for the given path
        """
        result = [offset]
        last = path[0]  # must be 0
        config = cls.get_drone_config(config_index)

        vertical_time = config.altitude * (1 / config.takeoff_speed + 1 / config.landing_speed)

        for index in path[1:]:
            if index == last:
                shift = 0.0
            else:
                shift = cls.drone_service_time[last] + vertical_time + cls.distances[last][index] / config.cruise_speed

            result.append(result[-1] + shift)
            last = index

        return tuple(result)

    @overload
    @classmethod
    def calculate_drone_total_waiting_time(
        cls,
        path: Sequence[int],
        *,
        arrival_timestamps: Tuple[float, ...],
    ) -> float: ...

    @overload
    @classmethod
    def calculate_drone_total_waiting_time(
        cls,
        path: Sequence[int],
        *,
        config_index: int,
        offset: float,
    ) -> float: ...

    @classmethod
    def calculate_drone_total_waiting_time(
        cls,
        path: Sequence[int],
        *,
        config_index: Optional[int] = None,
        offset: Optional[float] = None,
        arrival_timestamps: Optional[Tuple[float, ...]] = None,
    ) -> float:
        """Calculate the total waiting time of the given drone path

        Parameters
        -----
        path:
            The path to calculate
        config_index:
            The index of the drone config to use
        offset:
            The timestamp when the drone starts traveling
        arrival_timestamps:
            The arrival timestamps of the given path. If this is `None`, returned value from `calculate_drone_arrival_timestamps`
            will be used. Provide this argument can make the calculation faster.

        Returns
        -----
        The total waiting time of the given path
        """
        if arrival_timestamps is None:
            assert config_index is not None and offset is not None
            arrival_timestamps = cls.calculate_drone_arrival_timestamps(path, config_index=config_index, offset=offset)

        result = 0.0
        for path_index, index in enumerate(path[1:-1], start=1):
            result += arrival_timestamps[-1] - arrival_timestamps[path_index] - cls.drone_service_time[index]

        return result

    @classmethod
    def calculate_technician_arrival_timestamps(cls, path: Sequence[int], /) -> Tuple[float, ...]:
        """Calculate the arrival timestamps for the given technician path

        Parameters
        -----
        path:
            The path to calculate

        Returns
        -----
        The arrival timestamps for the given path
        """
        result = [0.0]
        last = path[0]  # must be 0
        config = cls.truck_config

        coefficients_iter = itertools.cycle(config.coefficients)
        current_within_timespan = 0.0
        velocity = config.maximum_velocity * next(coefficients_iter)

        for index in path[1:]:
            timestamp = result[-1] + cls.technician_service_time[last]
            current_within_timespan += cls.technician_service_time[last]
            while current_within_timespan >= 3600.0:
                current_within_timespan -= 3600.0
                velocity = config.maximum_velocity * next(coefficients_iter)

            distance = cls.distances[last][index]

            while distance > 0:
                time_shift = min(distance / velocity, 3600.0 - current_within_timespan)
                timestamp += time_shift
                distance -= time_shift * velocity
                current_within_timespan += time_shift

                if current_within_timespan >= 3600.0:
                    current_within_timespan = 0.0
                    velocity = config.maximum_velocity * next(coefficients_iter)

            result.append(timestamp)
            last = index

        return tuple(result)

    @classmethod
    def calculate_technician_total_waiting_time(cls, path: Sequence[int], *, arrival_timestamps: Optional[Tuple[float, ...]] = None) -> float:
        """Calculate the total waiting time of the given technician path

        Parameters
        -----
        path:
            The path to calculate
        arrival_timestamps:
            The arrival timestamps of the given path. If this is `None`, returned value from `calculate_technician_arrival_timestamps`
            will be used. Provide this argument can make the calculation faster.

        Returns
        -----
        The total waiting time of the given path
        """
        if arrival_timestamps is None:
            arrival_timestamps = cls.calculate_technician_arrival_timestamps(path)

        result = 0.0
        for path_index, index in enumerate(path[1:-1], start=1):
            result += arrival_timestamps[-1] - arrival_timestamps[path_index] - cls.technician_service_time[index]

        return result

    @classmethod
    def calculate_total_weight(cls, path: Sequence[int], /) -> float:
        """Calculate the total weight of all waypoints along the given path"""
        return sum(cls.demands[index] for index in path)

    @overload
    @classmethod
    def calculate_drone_flight_duration(
        cls,
        path: Sequence[int],
        *,
        arrival_timestamps: Tuple[float, ...],
    ) -> float: ...

    @overload
    @classmethod
    def calculate_drone_flight_duration(
        cls,
        path: Sequence[int],
        *,
        config_index: int,
        offset: float,
    ) -> float: ...

    @classmethod
    def calculate_drone_flight_duration(
        cls,
        path: Sequence[int],
        *,
        config_index: Optional[int] = None,
        offset: Optional[float] = None,
        arrival_timestamps: Optional[Tuple[float, ...]] = None,
    ) -> float:
        """Calculate the total flight duration of the given drone path

        Parameters
        -----
        path:
            The path to calculate
        config_index:
            The index of the drone config to use
        offset:
            The timestamp when the drone starts traveling
        arrival_timestamps:
            The arrival timestamps of the given path. If this is `None`, returned value from `calculate_drone_arrival_timestamps`
            will be used. Provide this argument can make the calculation faster.

        Returns
        -----
        The total flight duration of the given path
        """
        if arrival_timestamps is None:
            assert config_index is not None and offset is not None
            arrival_timestamps = cls.calculate_drone_arrival_timestamps(path, config_index=config_index, offset=offset)

        return arrival_timestamps[-1] - arrival_timestamps[0]

    @overload
    @classmethod
    def calculate_drone_energy_consumption(
        cls,
        path: Sequence[int],
        *,
        config_index: int,
        arrival_timestamps: Tuple[float, ...],
    ) -> float: ...

    @overload
    @classmethod
    def calculate_drone_energy_consumption(
        cls,
        path: Sequence[int],
        *,
        config_index: int,
        offset: float,
    ) -> float: ...

    @classmethod
    def calculate_drone_energy_consumption(
        cls,
        path: Sequence[int],
        *,
        config_index: int,
        offset: Optional[float] = None,
        arrival_timestamps: Optional[Tuple[float, ...]] = None,
    ) -> float:
        """Calculate the total energy consumption of the given drone path

        Parameters
        -----
        path:
            The path to calculate
        config_index:
            The index of the drone config to use
        offset:
            The timestamp when the drone starts traveling
        arrival_timestamps:
            The arrival timestamps of the given path. If this is `None`, returned value from `calculate_drone_arrival_timestamps`
            will be used. Provide this argument can make the calculation faster.

        Returns
        -----
        The total energy consumption of the given path
        """
        if arrival_timestamps is None:
            assert offset is not None
            arrival_timestamps = cls.calculate_drone_arrival_timestamps(path, config_index=config_index, offset=offset)

        config = cls.get_drone_config(config_index)

        takeoff_time = config.altitude / config.takeoff_speed
        landing_time = config.altitude / config.landing_speed

        result = weight = 0.0
        for path_index, index in enumerate(path[1:], start=1):
            last = path[path_index - 1]
            cruise_time = cls.distances[last][index] / config.cruise_speed
            result += (
                takeoff_time * config.takeoff_power(weight)
                + cruise_time * config.cruise_power(weight)
                + landing_time * config.landing_power(weight)
            )

            weight += cls.demands[index]

        return result

    @classmethod
    def initial(cls) -> D2DPathSolution:
        # Serve all technician-only waypoints
        technician_paths = [[0] for _ in range(cls.technicians_count)]
        technician_only = set(e for e in range(1, 1 + cls.customers_count) if not cls.dronable[e])

        technician_paths_iter = itertools.cycle(technician_paths)
        while len(technician_only) > 0:
            path = next(technician_paths_iter)
            index = min(technician_only, key=cls.distances[path[-1]].__getitem__)
            path.append(index)
            technician_only.remove(index)

        for path in technician_paths:
            path.append(0)

        # After this step, some technician paths may still be empty (i.e. [0, 0]), just leave them unchanged

        # Serve all dronable waypoints
        drone_paths = [[[0]] for _ in range(cls.drones_count)]
        dronable = set(e for e in range(1, 1 + cls.customers_count) if cls.dronable[e])

        drone_iter = itertools.cycle(range(cls.drones_count))
        while len(dronable) > 0:
            drone = next(drone_iter)
            paths = drone_paths[drone]
            config = cls.get_drone_config(cls.drone_config_mapping[drone])

            path = paths[-1]
            index = min(dronable, key=cls.distances[path[-1]].__getitem__)

            hypothetical_path = path + [index, 0]
            hypothetical_arrival_timestamps = cls.calculate_drone_arrival_timestamps(hypothetical_path, config_index=cls.drone_config_mapping[drone], offset=0.0)
            if (
                cls.calculate_total_weight(hypothetical_path) > config.capacity
                or cls.calculate_drone_energy_consumption(hypothetical_path, config_index=cls.drone_config_mapping[drone], arrival_timestamps=hypothetical_arrival_timestamps) > config.battery
            ):
                path.append(0)
                paths.append([0])

                technician_path = min(technician_paths, key=lambda path: cls.distances[index][path[-2]])
                technician_path.insert(-1, index)

            else:
                path.append(index)

            dronable.remove(index)

        for paths in drone_paths:
            paths[-1].append(0)

        return cls(
            drone_paths=tuple(tuple(tuple(path) for path in paths if len(path) > 2) for paths in drone_paths),
            technician_paths=tuple(tuple(path) for path in technician_paths),
        )

    @classmethod
    def get_drone_config(cls, config_index: int, /) -> Union[DroneLinearConfig, DroneNonlinearConfig]:
        if cls.energy_mode == DroneEnergyConsumptionMode.LINEAR:
            return cls.drone_linear_config[config_index]

        if cls.energy_mode == DroneEnergyConsumptionMode.NON_LINEAR:
            return cls.drone_nonlinear_config[config_index]

        raise RuntimeError("Shouldn't reach here")

    @classmethod
    def import_config(cls) -> None:
        cls.truck_config = TruckConfig.import_data()
        cls.drone_linear_config = DroneLinearConfig.import_data()
        cls.drone_nonlinear_config = DroneNonlinearConfig.import_data()
        cls.drone_endurance_config = DroneEnduranceConfig.import_data()

    @classmethod
    def share_distances(cls) -> _SharedDistancesManager:
        return _SharedDistancesManager()

    @classmethod
    def import_problem(
        cls,
        problem: str,
        *,
        drone_config_mapping: Tuple[int, ...],
        energy_mode: DroneEnergyConsumptionMode,
    ) -> None:
        if not cls.__config_imported:
            cls.import_config()
            cls.__config_imported = True

        try:
            problem = problem.removesuffix(".txt")
            path = join("problems", "d2d", "random_data", f"{problem}.txt")
            with open(path, "r") as file:
                data = file.read()

            cls.problem = problem
            cls.customers_count = int(re.search(r"Customers (\d+)", data).group(1))  # type: ignore
            cls.drones_count = int(re.search(r"number_drone (\d+)", data).group(1))  # type: ignore
            cls.technicians_count = int(re.search(r"number_drone (\d+)", data).group(1))  # type: ignore

            cls_x = [0.0]
            cls_y = [0.0]
            cls_demands = [0.0]
            cls_dronable = [True]
            cls_technician_service_time = [0.0]
            cls_drone_service_time = [0.0]
            for match in re.finditer(r"([-\d\.]+)\s+([-\d\.]+)\s+([\d\.]+)\s+(0|1)\t([\d\.]+)\s+([\d\.]+)", data):
                x, y, demand, technician_only, technician_service_time, drone_service_time = match.groups()
                cls_x.append(float(x))
                cls_y.append(float(y))
                cls_demands.append(float(demand))
                cls_dronable.append(technician_only == "0")
                cls_technician_service_time.append(float(technician_service_time))
                cls_drone_service_time.append(float(drone_service_time))

            cls.x = tuple(cls_x)
            cls.y = tuple(cls_y)
            cls.demands = tuple(cls_demands)
            cls.dronable = tuple(cls_dronable)
            cls.technician_service_time = tuple(cls_technician_service_time)
            cls.drone_service_time = tuple(cls_drone_service_time)

            cls.drone_config_mapping = drone_config_mapping
            cls.energy_mode = energy_mode

            try:
                memory = shared_memory.SharedMemory(name=problem, create=False)
                cls.distances = pickle.loads(memory.buf)

            except FileNotFoundError:
                distances = [[0.0] * (cls.customers_count + 1) for _ in range(cls.customers_count + 1)]
                for first, second in itertools.combinations(range(cls.customers_count + 1), 2):
                    distances[first][second] = distances[second][first] = sqrt((cls.x[first] - cls.x[second]) ** 2 + (cls.y[first] - cls.y[second]) ** 2)

                cls.distances = tuple(tuple(r) for r in distances)

        except Exception as e:
            raise ImportException(problem) from e

    def __hash__(self) -> int:
        return hash((self.drone_paths, self.technician_paths))

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, D2DPathSolution):
            return self.drone_paths == other.drone_paths and self.technician_paths == other.technician_paths

        return NotImplemented


class _SharedDistancesManager(contextlib.AbstractContextManager):

    __slots__ = (
        "memory",
    )
    if TYPE_CHECKING:
        memory: Final[shared_memory.SharedMemory]

    def __init__(self) -> None:
        data = pickle.dumps(D2DPathSolution.distances)
        self.memory = shared_memory.SharedMemory(name=D2DPathSolution.problem, create=True, size=len(data))
        self.memory.buf[:] = data

    def __exit__(self, *args: Any) -> Literal[False]:
        self.memory.unlink()
        return False
