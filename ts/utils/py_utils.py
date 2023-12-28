from __future__ import annotations

import itertools
import os
import platform
import threading
import sys
from typing import Any, Callable, Iterable, List, Optional, ParamSpec, Sequence, Set, Tuple, TypeVar, TYPE_CHECKING, overload

import numpy as np
from matplotlib import axes, pyplot
from pymoo.indicators.hv import HV  # type: ignore
from pymoo.indicators.igd import IGD  # type: ignore


__all__ = (
    "ngettext",
    "display_platform",
    "synchronized",
    "isclose",
    "hypervolume",
    "inverted_generational_distance",
    "plot_multi_fronts",
    "cost_dominate",
    "coverage_indicator",
    "build_pareto_front",
)
if TYPE_CHECKING:
    _P = ParamSpec("_P")
    _T = TypeVar("_T")
    _CostT = TypeVar("_CostT", bound=Sequence[float])


def ngettext(predicate: bool, if_true: str, if_false: str, /) -> str:
    return if_true if predicate else if_false


def display_platform() -> None:
    cpu_count = os.cpu_count() or 1

    display = f"Running on {sys.platform} with {cpu_count} " + ngettext(cpu_count == 1, "CPU", "CPUs") + "\n"
    display += f"Python {sys.version}\n"
    display += ", ".join((platform.platform(), platform.processor())) + "\n"
    display += "-" * 30

    print(display)


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
        return abs(first - second) < 0.0001


def normalize_costs(costs: Sequence[Tuple[float, float]], /) -> List[Tuple[float, float]]:
    min_costs = (min(cost[0] for cost in costs), min(cost[1] for cost in costs))
    max_costs = (max(cost[0] for cost in costs), max(cost[1] for cost in costs))

    def scale(cost: Tuple[float, float], /) -> Tuple[float, float]:
        try:
            x = (cost[0] - min_costs[0]) / (max_costs[0] - min_costs[0])
        except ZeroDivisionError:  # 0 / 0
            x = 1

        try:
            y = (cost[1] - min_costs[1]) / (max_costs[1] - min_costs[1])
        except ZeroDivisionError:  # 0 / 0
            y = 1

        return (x, y)

    return list(map(scale, costs))


def hypervolume(
    pareto_costs: Sequence[Tuple[float, float]],
    *,
    ref_point: Tuple[float, float],
) -> Optional[float]:
    pareto_costs = list(set(pareto_costs))
    pareto_costs.append(ref_point)
    *normalized, ref_normalized_point = normalize_costs(pareto_costs)

    indicator = HV(ref_point=ref_normalized_point)
    result = indicator(np.array(normalized))
    if result is None:
        message = f"Cannot calculate HV in hypervolume({pareto_costs!r}, ref_point={ref_point!r})"
        raise ValueError(message)

    return result


def inverted_generational_distance(
    pareto_costs: Sequence[Tuple[float, float]],
    *,
    ref_costs: Sequence[Tuple[float, float]],
) -> Optional[float]:
    pareto_costs = list(set(pareto_costs))
    ref_costs = list(set(ref_costs))

    merged = pareto_costs + ref_costs
    normalized = normalize_costs(merged)

    offset = len(pareto_costs)
    pareto_costs = normalized[:offset]
    ref_costs = normalized[offset:]

    indicator = IGD(np.array(ref_costs))
    result = indicator(np.array(pareto_costs))
    if result is None:
        message = f"Cannot calculate IGD in inverted_generational_distance({pareto_costs!r}, ref_costs={ref_costs!r})"
        raise ValueError(message)

    return result


def plot_multi_fronts(
    pareto_fronts: Iterable[Tuple[Iterable[Tuple[float, float]], str]],
    *,
    dump: Optional[str] = None,
    xlabel: str = "Objective 1",
    ylabel: str = "Objective 2",
) -> None:
    _, ax = pyplot.subplots()
    assert isinstance(ax, axes.Axes)

    markers = itertools.cycle(["s", "d", "x", "*", "2"])
    for index, (pareto_front, description) in enumerate(pareto_fronts):
        result_costs = set((round(r[0], 4), round(r[1], 4)) for r in pareto_front)
        ax.scatter(
            [cost[0] for cost in result_costs],
            [cost[1] for cost in result_costs],
            c=f"C{index}",
            marker=next(markers),
            label=description,
        )

    ax.grid(True)

    pyplot.xlabel(xlabel)
    pyplot.ylabel(ylabel)
    pyplot.legend()
    if dump is None:
        pyplot.show()
    else:
        pyplot.savefig(dump)

    pyplot.close()


def cost_dominate(first: _CostT, second: _CostT) -> bool:
    result = False
    for f, s in zip(first, second):
        if isclose(f, s):
            continue

        if f > s:
            return False

        if f < s:
            result = True

    return result


def coverage_indicator(first: Sequence[_CostT], second: Sequence[_CostT]) -> Tuple[float, float]:
    first_dominate = 0
    for f in first:
        if any(cost_dominate(f, s) for s in second):
            first_dominate += 1

    second_dominate = 0
    for s in second:
        if any(cost_dominate(s, f) for f in first):
            second_dominate += 1

    return first_dominate / len(first), second_dominate / len(second)


def build_pareto_front(costs: Iterable[_CostT]) -> Set[_CostT]:
    result: Set[_CostT] = set()
    for cost in costs:
        if any(cost_dominate(cost, c) for c in result):
            continue

        result = {c for c in result if not cost_dominate(c, cost)}
        result.add(cost)

    return result
