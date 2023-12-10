from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from math import cos, pi, sqrt
from os.path import join
from typing import Any, Dict, List, Literal, Tuple


__all__ = (
    "TruckConfig",
    "DroneLinearConfig",
    "DroneNonlinearConfig",
    "DroneEnduranceConfig",
)


@dataclass(frozen=True, kw_only=True, slots=True)
class TruckConfig:
    maximum_velocity: float
    m_t: float  # What the hell is this doing here?
    coefficients: Tuple[float, ...]

    @staticmethod
    def import_data() -> TruckConfig:
        path = join("problems", "d2d", "config_parameter", "Truck_config.json")
        with open(path, "r") as file:
            data = json.load(file)

        coefficients_d = data["T (hour)"]
        assert isinstance(coefficients_d, dict)

        return TruckConfig(
            maximum_velocity=data["V_max (m/s)"],
            m_t=data["M_t (kg)"],
            coefficients=tuple(coefficients_d.values()),
        )


@dataclass(frozen=True, kw_only=True, slots=True)
class _BaseDroneConfig:
    takeoff_speed: float
    cruise_speed: float
    landing_speed: float
    altitude: float
    capacity: float
    battery: float
    speed_type: Literal["low", "high"]
    range: Literal["low", "high"]

    def takeoff_power(self, weight: float, /) -> float:
        raise NotImplementedError

    def landing_power(self, weight: float, /) -> float:
        raise NotImplementedError

    def cruise_power(self, weight: float, /) -> float:
        raise NotImplementedError

    @staticmethod
    def from_data(data: Dict[str, Any]) -> _BaseDroneConfig:
        return _BaseDroneConfig(
            takeoff_speed=data["takeoffSpeed [m/s]"],
            cruise_speed=data["cruiseSpeed [m/s]"],
            landing_speed=data["landingSpeed [m/s]"],
            altitude=data["cruiseAlt [m]"],
            capacity=data["capacity [kg]"],
            battery=data["batteryPower [Joule]"],
            speed_type=data["speed_type"],
            range=data["range"],
        )


@dataclass(frozen=True, kw_only=True, slots=True)
class DroneLinearConfig(_BaseDroneConfig):
    beta: float
    gamma: float

    def _power(self, weight: float, /) -> float:
        return self.beta * weight + self.gamma

    def takeoff_power(self, weight: float, /) -> float:
        return self._power(weight)

    def landing_power(self, weight: float, /) -> float:
        return self._power(weight)

    def cruise_power(self, weight: float, /) -> float:
        return self._power(weight)

    @staticmethod
    def import_data() -> Tuple[DroneLinearConfig, ...]:
        path = join("problems", "d2d", "config_parameter", "drone_linear_config.json")
        with open(path, "r") as file:
            data = json.load(file)
            assert isinstance(data, dict)

        results: List[DroneLinearConfig] = []
        for d in data.values():
            base = _BaseDroneConfig.from_data(d)
            item = DroneLinearConfig(
                beta=d["beta(w/kg)"],
                gamma=d["gama(w)"],
                **asdict(base),
            )

            results.append(item)

        return tuple(results)


@dataclass(frozen=True, kw_only=True, slots=True)
class DroneNonlinearConfig(_BaseDroneConfig):
    k1: float
    k2: float
    c1: float
    c2: float
    c4: float
    c5: float

    def _vertical_power(self, speed: float, weight: float) -> float:
        w = 1.5 + weight
        g = 9.8
        return (
            self.k1 * w * g
            * (speed / 2 + sqrt((speed / 2) ** 2 + w * g / (self.k2 ** 2)))
            + self.c2 * ((w * g) ** 1.5)
        )

    def takeoff_power(self, weight: float, /) -> float:
        return self._vertical_power(self.takeoff_speed, weight)

    def landing_power(self, weight: float, /) -> float:
        return self._vertical_power(self.landing_speed, weight)

    def cruise_power(self, weight: float, /) -> float:
        w = 1.5 + weight
        g = 9.8
        return (
            (self.c1 + self.c2)
            * ((w * g - self.c5 * (self.cruise_speed * cos(pi / 18)) ** 2) ** 2 + (self.c4 * self.cruise_speed ** 2) ** 2) ** 0.75
            + self.c4 * self.cruise_speed ** 3
        )

    @staticmethod
    def import_data() -> Tuple[DroneNonlinearConfig, ...]:
        path = join("problems", "d2d", "config_parameter", "drone_nonelinear_config.json")
        with open(path, "r") as file:
            data = json.load(file)
            assert isinstance(data, dict)

        metadata = {
            "k1": data["k1"],
            "k2": data["k2 (sqrt(kg/m)"],
            "c1": data["c1 (sqrt(m/kg)"],
            "c2": data["c2 (sqrt(m/kg)"],
            "c4": data["c4 (kg/m)"],
            "c5": data["c5 (Ns/m)"],
        }

        results: List[DroneNonlinearConfig] = []
        for d in data.values():
            if isinstance(d, dict):
                base = _BaseDroneConfig.from_data(d)
                item = DroneNonlinearConfig(
                    **asdict(base),
                    **metadata,
                )

                results.append(item)

        return tuple(results)


@dataclass(frozen=True, kw_only=True, slots=True)
class DroneEnduranceConfig:
    speed_type: Literal["low", "high"]
    range: Literal["low", "high"]
    capacity: float
    fixed_time: float
    fixed_distance: float
    drone_speed: float

    @staticmethod
    def import_data() -> Tuple[DroneEnduranceConfig, ...]:
        path = join("problems", "d2d", "config_parameter", "drone_endurance_model_config.json")
        with open(path, "r") as file:
            data = json.load(file)
            assert isinstance(data, dict)

        results: List[DroneEnduranceConfig] = []
        for d in data.values():
            item = DroneEnduranceConfig(
                speed_type=d["speed_type"],
                range=d["range"],
                capacity=d["capacity [kg]"],
                fixed_time=d["FixedTime (s)"],
                fixed_distance=d["FixedDistance (m)"],
                drone_speed=d["Drone_speed (m/s)"],
            )

            results.append(item)

        return tuple(results)
