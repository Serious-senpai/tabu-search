from __future__ import annotations

import re
from math import sqrt
from os.path import join
from typing import ClassVar, Iterable, List, Tuple, TYPE_CHECKING, final

from .config import TruckConfig, DroneLinearConfig, DroneNonlinearConfig, DroneEnduranceConfig
from .errors import ImportException
from .mixins import SupportConstraintsMixin
from ..abc import BaseSolution


__all__ = (
    "D2DPathSolution",
)


@final
class D2DPathSolution(SupportConstraintsMixin, BaseSolution):
    """Represents a solution to the D2D problem"""

    __slots__ = (
        "paths",
    )
    __config_imported: ClassVar[bool] = False
    if TYPE_CHECKING:
        paths: Tuple[Tuple[Tuple[int, ...], ...], Tuple[Tuple[Tuple[int, ...], ...], ...]]

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
        truck_config: ClassVar[TruckConfig]
        drone_linear_config: ClassVar[Tuple[DroneLinearConfig, ...]]
        drone_nonlinear_config: ClassVar[Tuple[DroneNonlinearConfig, ...]]
        drone_endurance_config: ClassVar[Tuple[DroneEnduranceConfig, ...]]

    def __init__(
        self,
        *,
        drones: Iterable[Iterable[Tuple[int, ...]]],
        technicians: Iterable[Tuple[int, ...]],
        timespans: Tuple[Tuple[float, ...], Tuple[float, ...]],
        waiting_durations: Tuple[Tuple[float, ...], Tuple[float, ...]],
    ) -> None:
        super().__init__(timespans=timespans, waiting_durations=waiting_durations)
        self.paths = (
            tuple(technicians),
            tuple(map(tuple, drones)),
        )

    def distance(self, first: int, second: int, /) -> float:
        return sqrt((self.x[first] - self.x[second]) ** 2 + (self.y[first] - self.y[second]) ** 2)

    @classmethod
    def import_config(cls) -> None:
        cls.truck_config = TruckConfig.import_data()
        cls.drone_linear_config = DroneLinearConfig.import_data()
        cls.drone_nonlinear_config = DroneNonlinearConfig.import_data()
        cls.drone_endurance_config = DroneEnduranceConfig.import_data()

    @classmethod
    def import_data(cls, problem: str, /) -> None:
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

            cls_x: List[float] = []
            cls_y: List[float] = []
            cls_demands: List[float] = []
            cls_dronable: List[bool] = []
            cls_drone_service_time: List[float] = []
            cls_technician_service_time: List[float] = []
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

        except Exception as e:
            raise ImportException(problem) from e
