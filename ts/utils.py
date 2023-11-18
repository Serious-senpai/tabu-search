from __future__ import annotations

import platform
import math
import os
import threading
import sys
from typing import Any, Callable, List, Literal, Optional, ParamSpec, Sequence, Tuple, TypeVar, TYPE_CHECKING, overload

import numpy as np
from pymoo.indicators.hv import HV  # type: ignore
from pymoo.indicators.igd import IGD  # type: ignore


__all__ = (
    "false",
    "true",
    "zero",
    "ngettext",
    "display_platform",
    "synchronized",
    "isclose",
    "hypervolume",
    "inverted_generational_distance"
)


def false(*args: Any, **kwargs: Any) -> Literal[False]:
    return False


def true(*args: Any, **kwargs: Any) -> Literal[True]:
    return True


def zero(*args: Any, **kwargs: Any) -> Literal[0]:
    return 0


def ngettext(predicate: bool, if_true: str, if_false: str, /) -> str:
    return if_true if predicate else if_false


def display_platform() -> None:
    cpu_count = os.cpu_count() or 1

    display = f"Running on {sys.platform} with {cpu_count} " + ngettext(cpu_count == 1, "CPU", "CPUs") + "\n"
    display += f"Python {sys.version}\n"
    display += ", ".join((platform.platform(), platform.processor())) + "\n"
    display += "-" * 30

    print(display)


if TYPE_CHECKING:
    _P = ParamSpec("_P")
    _T = TypeVar("_T")


def synchronized(func: Callable[_P, _T], /) -> Callable[_P, _T]:
    lock = threading.Lock()

    def wrapper(*args: _P.args, **kwargs: _P.kwargs) -> _T:
        with lock:
            return func(*args, **kwargs)

    return wrapper


@overload
def isclose(
    first: float,
    second: float,
    /,
) -> bool: ...


@overload
def isclose(
    first: Sequence[float],
    second: Sequence[float],
    /,
) -> bool: ...


@overload
def isclose(
    first: Sequence[Sequence[float]],
    second: Sequence[Sequence[float]],
    /,
) -> bool: ...


@overload
def isclose(
    first: Sequence[Sequence[Sequence[float]]],
    second: Sequence[Sequence[Sequence[float]]],
    /,
) -> bool: ...


@overload
def isclose(
    first: Sequence[Sequence[Sequence[Sequence[float]]]],
    second: Sequence[Sequence[Sequence[Sequence[float]]]],
    /,
) -> bool: ...


@overload
def isclose(
    first: Sequence[Sequence[Sequence[Sequence[Sequence[float]]]]],
    second: Sequence[Sequence[Sequence[Sequence[Sequence[float]]]]],
    /,
) -> bool: ...


def isclose(first: Any, second: Any, /) -> bool:
    try:
        return all(isclose(f, s) for f, s in zip(first, second))
    except TypeError:
        return math.isclose(first, second, rel_tol=0.001, abs_tol=0.001)


def normalize_costs(costs: Sequence[Tuple[float, float]], /) -> List[Tuple[float, float]]:
    min_costs = (min(cost[0] for cost in costs), min(cost[1] for cost in costs))
    max_costs = (max(cost[0] for cost in costs), max(cost[1] for cost in costs))

    def scale(cost: Tuple[float, float], /) -> Tuple[float, float]:
        return (
            (cost[0] - min_costs[0]) / (max_costs[0] - min_costs[0]),
            (cost[1] - min_costs[1]) / (max_costs[1] - min_costs[1]),
        )

    return list(map(scale, costs))


def hypervolume(
    pareto_costs: Sequence[Tuple[float, float]],
    *,
    ref_normalized_point: Tuple[float, float],
) -> Optional[float]:
    indicator = HV(ref_point=ref_normalized_point)
    return indicator(np.array(normalize_costs(pareto_costs)))


def inverted_generational_distance(
    pareto_costs: Sequence[Tuple[float, float]],
    *,
    ref_costs: Sequence[Tuple[float, float]],
) -> Optional[float]:
    merged = list(pareto_costs) + list(ref_costs)
    normalized = normalize_costs(merged)

    offset = len(pareto_costs)
    pareto_costs = normalized[:offset]
    ref_costs = normalized[offset:]

    indicator = IGD(np.array(ref_costs))
    return indicator(np.array(pareto_costs))
