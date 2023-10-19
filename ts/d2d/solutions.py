from __future__ import annotations

import itertools
import re
from enum import Enum
from functools import partial
from math import sqrt
from os.path import join
from typing import ClassVar, Iterable, List, Optional, Tuple, Union, TYPE_CHECKING, final

from matplotlib import axes, pyplot

from .config import TruckConfig, DroneLinearConfig, DroneNonlinearConfig, DroneEnduranceConfig
from .errors import ImportException
from .mixins import SolutionMetricsMixin
from ..abc import BaseSolution


__all__ = (
    "DroneEnergyConsumptionMode",
    "D2DPathSolution",
)


class DroneEnergyConsumptionMode(Enum):
    NON_LINEAR = 0
    LINEAR = 1


@final
class D2DPathSolution(SolutionMetricsMixin, BaseSolution):
    """Represents a solution to the D2D problem"""

    __slots__ = (
        "drone_paths",
        "technician_paths",
        "drone_arrival_timestamps",
        "technician_arrival_timestamps",
        "drone_energy_consumption",
    )
    __config_imported: ClassVar[bool] = False
    if TYPE_CHECKING:
        drone_paths: Tuple[Tuple[Tuple[int, ...], ...], ...]
        technician_paths: Tuple[Tuple[int, ...], ...]

        drone_arrival_timestamps: Tuple[Tuple[Tuple[float, ...], ...], ...]
        technician_arrival_timestamps: Tuple[Tuple[float, ...], ...]

        drone_energy_consumption: Tuple[Tuple[Tuple[float, ...], ...], ...]

        # Problem-specific data
        problem: ClassVar[str]
        customers_count: ClassVar[int]
        drones_count: ClassVar[int]
        technicians_count: ClassVar[int]
        drones_flight_duration: ClassVar[float]

        x: ClassVar[Tuple[float, ...]]
        y: ClassVar[Tuple[float, ...]]
        demands: ClassVar[Tuple[float, ...]]
        dronable: ClassVar[Tuple[bool, ...]]
        drone_service_time: ClassVar[Tuple[float, ...]]
        technician_service_time: ClassVar[Tuple[float, ...]]

        # Global configuration data
        energy_mode: ClassVar[DroneEnergyConsumptionMode]
        truck_config: ClassVar[TruckConfig]
        drone_linear_config: ClassVar[Tuple[DroneLinearConfig, ...]]
        drone_nonlinear_config: ClassVar[Tuple[DroneNonlinearConfig, ...]]
        drone_endurance_config: ClassVar[Tuple[DroneEnduranceConfig, ...]]

    def __init__(
        self,
        *,
        drone_paths: Iterable[Iterable[Tuple[int, ...]]],
        technician_paths: Iterable[Tuple[int, ...]],
        drone_arrival_timestamps: Optional[Tuple[Tuple[Tuple[float, ...], ...], ...]] = None,
        technician_arrival_timestamps: Optional[Tuple[Tuple[float, ...], ...]] = None,
        drone_energy_consumption: Optional[Tuple[Tuple[Tuple[float, ...], ...], ...]] = None,
        timespan: Optional[float] = None,
        total_waiting_time: Optional[float] = None,
    ) -> None:
        self.drone_paths = tuple(map(tuple, drone_paths))
        self.technician_paths = tuple(technician_paths)

        if drone_arrival_timestamps is None:
            self.drone_arrival_timestamps = self._calculate_drone_arrival_timestamps()
        else:
            self.drone_arrival_timestamps = drone_arrival_timestamps

        if technician_arrival_timestamps is None:
            self.technician_arrival_timestamps = self._calculate_technician_arrival_timestamps()
        else:
            self.technician_arrival_timestamps = technician_arrival_timestamps

        if drone_energy_consumption is None:
            self.drone_energy_consumption = self._calculate_drone_energy_consumption()
        else:
            self.drone_energy_consumption = drone_energy_consumption

        if timespan is None:
            timespan = 0.0
            for paths in self.drone_arrival_timestamps:
                timespan = max(timespan, max(path[-1] for path in paths))

            for path in self.technician_arrival_timestamps:
                timespan = max(timespan, path[-1])

        if total_waiting_time is None:
            total_waiting_time = 0.0
            for drone, paths in enumerate(self.drone_arrival_timestamps):
                for path_index, path in enumerate(paths):
                    for index, timestamp in enumerate(path):
                        total_waiting_time += path[-1] - timestamp - self.drone_service_time[self.drone_paths[drone][path_index][index]]

            for technician, path in enumerate(self.technician_arrival_timestamps):
                for index, timestamp in enumerate(path):
                    total_waiting_time += path[-1] - timestamp - self.technician_service_time[self.technician_paths[technician][index]]

        super().__init__(
            timespan=timespan,
            total_waiting_time=total_waiting_time,
        )

    def _calculate_drone_arrival_timestamps(self) -> Tuple[Tuple[Tuple[float, ...], ...], ...]:
        result: List[Tuple[Tuple[float, ...], ...]] = []
        for drone, paths in enumerate(self.drone_paths):
            config: Union[DroneLinearConfig, DroneNonlinearConfig] = self.drone_linear_config[drone]
            if self.energy_mode == DroneEnergyConsumptionMode.NON_LINEAR:
                config = self.drone_nonlinear_config[drone]

            vertical_time = config.altitude * (1 / config.takeoff_speed + 1 / config.landing_speed)

            timestamp = 0.0
            last: Optional[int] = None
            drone_arrivals: List[Tuple[float, ...]] = []
            for path in paths:
                arrival: List[float] = []
                for index in path:
                    if last is not None:
                        timestamp += vertical_time + self.distance(last, index) / config.cruise_speed

                    arrival.append(timestamp)
                    timestamp += self.drone_service_time[index]

                    last = index

                drone_arrivals.append(tuple(arrival))

            result.append(tuple(drone_arrivals))

        return tuple(result)

    def _calculate_technician_arrival_timestamps(self) -> Tuple[Tuple[float, ...], ...]:
        result: List[Tuple[float, ...]] = []
        config = self.truck_config
        maximum_velocity_iter = iter(config.maximum_velocity * e for e in config.coefficients)

        for path in self.technician_paths:
            timestamp = 0.0
            last: Optional[int] = None
            arrival: List[float] = []
            current_time_within_window = 0.0
            current_velocity = next(maximum_velocity_iter)
            for index in path:
                if last is not None:
                    distance = self.distance(last, index)
                    while distance > 0:
                        time_shift = min(3600 - current_time_within_window, distance / current_velocity)
                        timestamp += time_shift
                        current_time_within_window += time_shift
                        if current_time_within_window >= 3600:
                            current_time_within_window = 0.0
                            current_velocity = next(maximum_velocity_iter)

                        distance -= time_shift * current_velocity

                arrival.append(timestamp)
                timestamp += self.technician_service_time[index]
                last = index

            result.append(tuple(arrival))

        return tuple(result)

    def _calculate_drone_energy_consumption(self) -> Tuple[Tuple[Tuple[float, ...], ...], ...]:
        result: List[Tuple[Tuple[float, ...], ...]] = []
        for drone, paths in enumerate(self.drone_paths):
            config: Union[DroneLinearConfig, DroneNonlinearConfig] = self.drone_linear_config[drone]
            if self.energy_mode == DroneEnergyConsumptionMode.NON_LINEAR:
                config = self.drone_nonlinear_config[drone]

            vertical_time = config.altitude * (1 / config.takeoff_speed + 1 / config.landing_speed)

            drone_consumption: List[Tuple[float, ...]] = []
            for path in paths:
                consumption = [0.0]
                weight = 0.0
                for path_index, index in enumerate(path[1:], start=1):
                    last = path[path_index - 1]
                    weight += self.demands[last]
                    time = vertical_time + self.distance(last, index) / config.cruise_speed
                    if isinstance(config, DroneLinearConfig):
                        energy = time * config.power(weight)

                    else:
                        energy = time * (config.takeoff_power(weight) + config.landing_power(weight) + config.cruise_power(weight))

                    consumption.append(consumption[-1] + energy)

                drone_consumption.append(tuple(consumption))

            result.append(tuple(drone_consumption))

        return tuple(result)

    def plot(self) -> None:
        _, ax = pyplot.subplots()
        assert isinstance(ax, axes.Axes)

        colors = (
            "red",
            "green",
            "cyan",
            "darkblue",
            "darkviolet",
            "violet",
            "gray",
            "dodgerblue",
        )
        colors_iter = itertools.cycle(colors)
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
                color=next(colors_iter),
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
                color=next(colors_iter),
                angles="xy",
                scale_units="xy",
                scale=1,
            )

        ax.scatter((0,), (0,), c="black", label="Deport")
        ax.scatter(
            [self.x[index] for index in range(1, 1 + self.customers_count) if self.dronable[index]],
            [self.y[index] for index in range(1, 1 + self.customers_count) if self.dronable[index]],
            c=next(colors_iter),
            label="Dronable",
        )
        ax.scatter(
            [self.x[index] for index in range(1, 1 + self.customers_count) if not self.dronable[index]],
            [self.y[index] for index in range(1, 1 + self.customers_count) if not self.dronable[index]],
            c=next(colors_iter),
            label="Technician-only",
        )

        ax.annotate("0", (0, 0))
        for index in range(1, 1 + self.customers_count):
            ax.annotate(str(index), (self.x[index], self.y[index]))

        ax.grid(True)

        pyplot.legend()
        pyplot.show()

    @classmethod
    def distance(cls, first: int, second: int, /) -> float:
        return sqrt((cls.x[first] - cls.x[second]) ** 2 + (cls.y[first] - cls.y[second]) ** 2)

    @classmethod
    def initial(cls) -> D2DPathSolution:
        technician_paths = [[0] for _ in range(cls.technicians_count)]
        technician_only = set(e for e in range(1, 1 + cls.customers_count) if not cls.dronable[e])

        technician_iter = itertools.cycle(technician_paths)
        while len(technician_only) > 0:
            path = next(technician_iter)
            index = min(technician_only, key=partial(cls.distance, path[-1]))
            path.append(index)
            technician_only.remove(index)

        for path in technician_paths:
            path.append(0)

        drone_paths = [[[0]] for _ in range(cls.drones_count)]
        weight = [0.0] * cls.drones_count
        time = [0.0] * cls.drones_count
        energy = [0.0] * cls.drones_count
        dronable = set(e for e in range(1, 1 + cls.customers_count) if cls.dronable[e])

        drone_iter = itertools.cycle(range(cls.drones_count))
        while len(dronable) > 0:
            drone = next(drone_iter)
            paths = drone_paths[drone]

            config: Union[DroneLinearConfig, DroneNonlinearConfig] = cls.drone_linear_config[drone]
            if cls.energy_mode == DroneEnergyConsumptionMode.NON_LINEAR:
                config = cls.drone_nonlinear_config[drone]

            last = paths[-1][-1]
            index = min(dronable, key=partial(cls.distance, last))

            dw = cls.demands[index]
            dt = config.altitude * (1 / config.takeoff_speed + 1 / config.landing_speed) + cls.distance(last, index) / config.cruise_speed
            if isinstance(config, DroneLinearConfig):
                de = dt * config.power(weight[drone])
            else:
                de = dt * (config.takeoff_power(weight[drone]) + config.landing_power(weight[drone]) + config.cruise_power(weight[drone]))

            if weight[drone] + dw <= config.capacity and time[drone] + dt <= cls.drones_flight_duration and energy[drone] + de <= config.battery:
                paths[-1].append(index)
                weight[drone] = weight[drone] + dw
                time[drone] = time[drone] + dt
                energy[drone] = energy[drone] + de

            else:
                paths[-1].append(0)
                paths.append([0])

                technician_path = min(technician_paths, key=lambda path: cls.distance(index, path[-2]))
                technician_path.insert(-1, index)

            dronable.remove(index)

        return cls(
            drone_paths=tuple(tuple(tuple(path) for path in paths if len(path) > 2) for paths in drone_paths),
            technician_paths=tuple(tuple(path) for path in technician_paths if len(path) > 2),
        )

    @classmethod
    def import_config(cls) -> None:
        cls.truck_config = TruckConfig.import_data()
        cls.drone_linear_config = DroneLinearConfig.import_data()
        cls.drone_nonlinear_config = DroneNonlinearConfig.import_data()
        cls.drone_endurance_config = DroneEnduranceConfig.import_data()

    @classmethod
    def import_problem(cls, problem: str, *, energy_mode: DroneEnergyConsumptionMode = DroneEnergyConsumptionMode.LINEAR) -> None:
        if not cls.__config_imported:
            cls.import_config()
            cls.__config_imported = True

        try:
            path = join("problems", "d2d", "random_data", f"{problem}.txt")
            with open(path, "r") as file:
                data = file.read()

            cls.problem = problem
            cls.customers_count = int(re.search(r"Customers (\d+)", data).group(1))  # type: ignore
            cls.drones_count = int(re.search(r"number_drone (\d+)", data).group(1))  # type: ignore
            cls.technicians_count = int(re.search(r"number_drone (\d+)", data).group(1))  # type: ignore
            cls.drones_flight_duration = float(re.search(r"droneLimitationFightTime\(s\) (\d+)", data).group(1))  # type: ignore

            cls_x = [0.0]
            cls_y = [0.0]
            cls_demands = [0.0]
            cls_dronable = [True]
            cls_drone_service_time = [0.0]
            cls_technician_service_time = [0.0]
            for match in re.finditer(r"([-\d\.]+)\s+([-\d\.]+)\s+([\d\.]+)\s+(0|1)\t([\d\.]+)\s+([\d\.]+)", data):
                x, y, demand, technician_only, drone_service_time, technician_service_time = match.groups()
                cls_x.append(float(x))
                cls_y.append(float(y))
                cls_demands.append(float(demand))
                cls_dronable.append(technician_only == "0")
                cls_drone_service_time.append(float(drone_service_time))
                cls_technician_service_time.append(float(technician_service_time))

            cls.x = tuple(cls_x)
            cls.y = tuple(cls_y)
            cls.demands = tuple(cls_demands)
            cls.dronable = tuple(cls_dronable)
            cls.drone_service_time = tuple(cls_drone_service_time)
            cls.technician_service_time = tuple(cls_technician_service_time)

            cls.energy_mode = energy_mode

        except Exception as e:
            raise ImportException(problem) from e
