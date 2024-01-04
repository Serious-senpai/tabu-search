from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Literal, Optional, Tuple, TYPE_CHECKING

from ts import d2d, utils


# Energy modes
LINEAR = "linear"
NON_LINEAR = "non-linear"
ENDURANCE = "endurance"


# Propagation priority
NONE = "none"
MIN_DISTANCE = "min-distance"
MAX_DISTANCE = "max-distance"
IDEAL_DISTANCE = "ideal-distance"
MIN_DISTANCE_NO_NORMALIZE = "min-distance-no-normalize"
MAX_DISTANCE_NO_NORMALIZE = "max-distance-no-normalize"
IDEAL_DISTANCE_NO_NORMALIZE = "ideal-distance-no-normalize"


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: str
        iterations: int
        tabu_size: int
        drone_config: int
        energy_mode: Literal["linear", "non-linear", "endurance"]
        propagation_priority: Literal[
            "none",
            "min-distance",
            "max-distance",
            "ideal-distance",
            "min-distance-no-normalize",
            "max-distance-no-normalize",
            "ideal-distance-no-normalize",
        ]
        max_propagation: int
        verbose: bool
        dump: Optional[str]
        extra: Optional[str]
        pool_size: int


def to_json(solution: d2d.D2DPathSolution) -> Dict[str, Any]:
    return {
        "cost": solution.cost(),
        "drone_paths": solution.drone_paths,
        "technician_paths": solution.technician_paths,
    }


def normalization(value: float, minimum: float, maximum: float) -> float:
    try:
        return value / (maximum - minimum)
    except ZeroDivisionError:
        if not utils.isclose(value, 0.0):
            message = f"Called with normalization({value}, {minimum}, {maximum})"
            raise ValueError(message)

        return 0.0


def _max_distance_key_no_normalize(
    pareto_costs: Dict[Tuple[float, ...], int],
    minimum: Tuple[float, ...],
    maximum: Tuple[float, ...],
    candidate: d2d.D2DPathSolution,
    /
) -> float:
    cost = candidate.cost()
    result = 0.0
    for pareto_cost, counter in pareto_costs.items():
        result += counter * abs(pareto_cost[0] - cost[0]) + abs(pareto_cost[1] - cost[1])

    return -result


def _max_distance_key(
    pareto_costs: Dict[Tuple[float, ...], int],
    minimum: Tuple[float, ...],
    maximum: Tuple[float, ...],
    candidate: d2d.D2DPathSolution,
    /
) -> float:
    cost = candidate.cost()
    result = 0.0
    for pareto_cost, counter in pareto_costs.items():
        result += counter * (
            normalization(abs(pareto_cost[0] - cost[0]), minimum[0], maximum[0])
            + normalization(abs(pareto_cost[1] - cost[1]), minimum[1], maximum[1])
        )

    return -result


def _min_distance_key_no_normalize(
    pareto_costs: Dict[Tuple[float, ...], int],
    minimum: Tuple[float, ...],
    maximum: Tuple[float, ...],
    candidate: d2d.D2DPathSolution,
    /
) -> float:
    cost = candidate.cost()
    result = 0.0
    for pareto_cost, counter in pareto_costs.items():
        result += counter * abs(pareto_cost[0] - cost[0]) + abs(pareto_cost[1] - cost[1])

    return result


def _min_distance_key(
    pareto_costs: Dict[Tuple[float, ...], int],
    minimum: Tuple[float, ...],
    maximum: Tuple[float, ...],
    candidate: d2d.D2DPathSolution,
    /
) -> float:
    cost = candidate.cost()
    result = 0.0
    for pareto_cost, counter in pareto_costs.items():
        result += counter * (
            normalization(abs(pareto_cost[0] - cost[0]), minimum[0], maximum[0])
            + normalization(abs(pareto_cost[1] - cost[1]), minimum[1], maximum[1])
        )

    return result


def _ideal_distance_key_no_normalize(
    pareto_costs: Dict[Tuple[float, ...], int],
    minimum: Tuple[float, ...],
    maximum: Tuple[float, ...],
    candidate: d2d.D2DPathSolution,
    /
) -> float:
    ideal = (min(cost[0] for cost in pareto_costs.keys()), min(cost[1] for cost in pareto_costs.keys()))
    cost = candidate.cost()
    return abs(ideal[0] - cost[0]) + abs(ideal[1] - cost[1])


def _ideal_distance_key(
    pareto_costs: Dict[Tuple[float, ...], int],
    minimum: Tuple[float, ...],
    maximum: Tuple[float, ...],
    candidate: d2d.D2DPathSolution,
    /
) -> float:
    ideal = (min(cost[0] for cost in pareto_costs.keys()), min(cost[1] for cost in pareto_costs.keys()))
    cost = candidate.cost()
    return (
        normalization(abs(ideal[0] - cost[0]), minimum[0], maximum[0])
        + normalization(abs(ideal[1] - cost[1]), minimum[1], maximum[1])
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tabu search algorithm for D2D problems")
    parser.add_argument("problem", type=str, help="the problem name (e.g. \"6.5.1\", \"200.10.1\", ...)")
    parser.add_argument("-i", "--iterations", default=1500, type=int, help="the number of iterations to run the tabu search for (default: 1500)")
    parser.add_argument("-t", "--tabu-size", default=10, type=int, help="the tabu size for every neighborhood (default: 10)")
    parser.add_argument("-c", "--drone-config", default=0, type=int, help="the energy configuration index for each drone (default: 0)")
    parser.add_argument("-e", "--energy-mode", default=LINEAR, choices=[LINEAR, NON_LINEAR, ENDURANCE], help="the energy consumption mode to use (default: linear)")
    parser.add_argument(
        "-k",
        "--propagation-priority",
        default=MIN_DISTANCE,
        choices=[NONE, MIN_DISTANCE, MAX_DISTANCE, IDEAL_DISTANCE, MIN_DISTANCE_NO_NORMALIZE, MAX_DISTANCE_NO_NORMALIZE, IDEAL_DISTANCE_NO_NORMALIZE],
        help="set the solution propagation priority (default: none)",
    )
    parser.add_argument("-m", "--max-propagation", default=5, type=int, help="maximum number of propagating solutions at a time (default: 5)")
    parser.add_argument("-v", "--verbose", action="store_true", help="whether to display the progress bar and plot the solution")
    parser.add_argument("-d", "--dump", type=str, help="dump the solution to a file")

    parser.add_argument("--extra", type=str, help="extra data dump to file specified by --dump")

    default_pool_size = os.cpu_count() or 1
    parser.add_argument("--pool-size", default=default_pool_size, type=int, help=f"the size of the process pool (default: {default_pool_size})")

    utils.display_platform()

    namespace = Namespace()
    parser.parse_args(namespace=namespace)
    print(namespace)

    d2d.D2DPathSolution.import_problem(
        namespace.problem,
        drone_config=namespace.drone_config,
        energy_mode=namespace.energy_mode,
    )
    d2d.Swap.reset_tabu(maxlen=namespace.tabu_size)
    d2d.Insert.reset_tabu(maxlen=namespace.tabu_size)

    propagation_priority_key: Optional[Callable[[Dict[Tuple[float, ...], int], Tuple[float, ...], Tuple[float, ...], d2d.D2DPathSolution], float]] = None
    if namespace.propagation_priority == MIN_DISTANCE:
        propagation_priority_key = _min_distance_key
    elif namespace.propagation_priority == MAX_DISTANCE:
        propagation_priority_key = _max_distance_key
    elif namespace.propagation_priority == IDEAL_DISTANCE:
        propagation_priority_key = _ideal_distance_key
    elif namespace.propagation_priority == MIN_DISTANCE_NO_NORMALIZE:
        propagation_priority_key = _min_distance_key_no_normalize
    elif namespace.propagation_priority == MAX_DISTANCE_NO_NORMALIZE:
        propagation_priority_key = _max_distance_key_no_normalize
    elif namespace.propagation_priority == IDEAL_DISTANCE_NO_NORMALIZE:
        propagation_priority_key = _ideal_distance_key_no_normalize

    start = time.perf_counter()
    solutions = sorted(
        d2d.D2DPathSolution.tabu_search(
            pool_size=namespace.pool_size,
            iterations_count=namespace.iterations,
            use_tqdm=namespace.verbose,
            propagation_priority_key=propagation_priority_key,
            max_propagation=namespace.max_propagation,
            plot_pareto_front=namespace.verbose,
        ),
        key=lambda s: s.cost(),
    )
    total = time.perf_counter() - start

    costs = [s.cost() for s in solutions]
    hv_ref = max(cost[0] for cost in costs), max(cost[1] for cost in costs)

    print(f"Found {len(solutions)} " + utils.ngettext(len(solutions) == 1, "solution", "solutions"))
    errors: List[str] = []
    for index, solution in enumerate(solutions):
        print(f"SOLUTION #{index + 1}: cost = {solution.cost()}")
        print("\n".join(f"Drone #{drone_index + 1}: {paths}" for drone_index, paths in enumerate(solution.drone_paths)))
        print("\n".join(f"Technician #{technician_index + 1}: {path}" for technician_index, path in enumerate(solution.technician_paths)))

        errors_messages: List[str] = []
        if not solution.feasible():
            errors_messages.append("Solution is infeasible")

        check = d2d.D2DPathSolution(
            drone_paths=solution.drone_paths,
            technician_paths=solution.technician_paths,
        )

        if not utils.isclose(check.cost(), solution.cost()):
            errors_messages.append(f"Incorrect solution cost: Expected {check.cost()}, got {solution.cost()}")

        for attr in (
            "drone_timespans",
            "technician_timespans",
            "drone_waiting_times",
            "technician_waiting_times",
        ):
            check_attr = getattr(check, attr)
            solution_attr = getattr(solution, attr)
            if not utils.isclose(check_attr, solution_attr):
                errors_messages.append(f"Incorrect {attr}: Expected {check_attr}, got {solution_attr}")

        if len(errors_messages) > 0:
            errors.append(f"At solution #{index + 1}:")
            errors.extend(errors_messages)

    if namespace.dump is not None:
        dump_path = Path(namespace.dump)
        dump_path.parent.mkdir(parents=True, exist_ok=True)
        with dump_path.open("w", encoding="utf-8") as f:
            data = {
                "problem": namespace.problem,
                "iterations": namespace.iterations,
                "tabu_size": namespace.tabu_size,
                "drone_config": namespace.drone_config,
                "energy_mode": namespace.energy_mode,
                "propagation_priority": namespace.propagation_priority,
                "solutions": [to_json(s) for s in solutions],
                "extra": namespace.extra,
                "last_improved": d2d.D2DPathSolution.tabu_search_last_improved,
                "time": total,
            }
            json.dump(data, f)

        print(f"Saved solution to {namespace.dump!r}")

    if len(errors) > 0:
        raise ValueError("Some calculations were incorrect:\n" + "\n".join(errors))
