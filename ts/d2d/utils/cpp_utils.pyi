from typing import List, Literal, Sequence


__all__ = (
    "import_truck_config",
    "import_drone_linear_config",
    "import_drone_nonlinear_config",
    "import_drone_endurance_config",
    "import_customers",
    "calculate_drone_arrival_timestamps",
    "calculate_technician_arrival_timestamps",
    "calculate_drone_total_waiting_time",
    "calculate_technician_total_waiting_time",
)


def import_truck_config(
    *,
    maximum_velocity: float,
    m_t: float,
    coefficients: Sequence[float],
) -> None: ...


def import_drone_linear_config(
    *,
    takeoff_speed: float,
    cruise_speed: float,
    landing_speed: float,
    altitude: float,
    capacity: float,
    battery: float,
    speed_type: Literal["low", "high"],
    range: Literal["low", "high"],
    beta: float,
    gamma: float,
) -> None: ...


def import_drone_nonlinear_config(
    *,
    takeoff_speed: float,
    cruise_speed: float,
    landing_speed: float,
    altitude: float,
    capacity: float,
    battery: float,
    speed_type: Literal["low", "high"],
    range: Literal["low", "high"],
    k1: float,
    k2: float,
    c1: float,
    c2: float,
    c4: float,
    c5: float,
) -> None: ...


def import_drone_endurance_config(
    *,
    speed_type: Literal["low", "high"],
    range: Literal["low", "high"],
    capacity: float,
    fixed_time: float,
    fixed_distance: float,
    drone_speed: float,
) -> None: ...


def import_customers(
    *,
    x: Sequence[float],
    y: Sequence[float],
    demands: Sequence[float],
    dronable: Sequence[bool],
    drone_service_time: Sequence[float],
    technician_service_time: Sequence[float],
) -> None: ...


def calculate_drone_arrival_timestamps(
    path: Sequence[int],
    *,
    config_type: Literal[0, 1, 2],
    offset: float,
) -> List[float]: ...


def calculate_technician_arrival_timestamps(
    path: Sequence[int],
) -> List[float]: ...


def calculate_drone_total_waiting_time(
    path: Sequence[int],
    *,
    arrival_timestamps: Sequence[float],
) -> float: ...


def calculate_technician_total_waiting_time(
    path: Sequence[int],
    *,
    arrival_timestamps: Sequence[float],
) -> float: ...
