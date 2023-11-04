from __future__ import annotations

import argparse
import cProfile
import json
import os
from typing import Any, Callable, Dict, Literal, Optional, Set, TYPE_CHECKING

from ts import d2d, utils


LINEAR = "linear"
NON_LINEAR = "non-linear"


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: str
        iterations: int
        tabu_size: int
        energy_mode: Literal["linear", "non-linear"]
        max_distance: bool
        min_distance: bool
        max_propagation: Optional[int]
        profile: bool
        verbose: bool
        dump: Optional[str]
        pool_size: int


def to_json(solution: d2d.D2DPathSolution) -> Dict[str, Any]:
    return {
        "cost": solution.cost(),
        "drone_config_mapping": solution.drone_config_mapping,
        "drone_paths": solution.drone_paths,
        "technician_paths": solution.technician_paths,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tabu search algorithm for TSP problems")
    parser.add_argument("problem", type=str, help="the problem name (e.g. \"6.5.1\", \"200.10.1\", ...)")
    parser.add_argument("-i", "--iterations", default=500, type=int, help="the number of iterations to run the tabu search for (default: 500)")
    parser.add_argument("-t", "--tabu-size", default=10, type=int, help="the tabu size for every neighborhood (default: 10)")
    parser.add_argument("-e", "--energy-mode", default=LINEAR, choices=[LINEAR, NON_LINEAR], help="the energy consumption mode to use (default: linear)")
    parser.add_argument("--max-distance", action="store_true", help="set the propagation predicate using the maximum total distance to the Pareto front instead of the propagation rate")
    parser.add_argument("--min-distance", action="store_true", help="set the propagation predicate using the minimum total distance to the Pareto front instead of the propagation rate")
    parser.add_argument("-m", "--max-propagation", type=int, help="maximum number of propagating solutions at a time")
    parser.add_argument("-p", "--profile", action="store_true", help="run in profile mode and exit immediately")
    parser.add_argument("-v", "--verbose", action="store_true", help="whether to display the progress bar and plot the solution")
    parser.add_argument("-d", "--dump", type=str, help="dump the solution to a file")

    default_pool_size = os.cpu_count() or 1
    parser.add_argument("--pool-size", default=default_pool_size, type=int, help=f"the size of the process pool (default: {default_pool_size})")

    utils.display_platform()

    namespace: Namespace = parser.parse_args()  # type: ignore
    print(namespace)

    if namespace.energy_mode == LINEAR:
        energy_mode = d2d.DroneEnergyConsumptionMode.LINEAR
    elif namespace.energy_mode == NON_LINEAR:
        energy_mode = d2d.DroneEnergyConsumptionMode.NON_LINEAR
    else:
        raise ValueError(f"Unknown energy mode {namespace.energy_mode!r}")

    d2d.D2DPathSolution.import_problem(namespace.problem, energy_mode=energy_mode)
    d2d.Swap.reset_tabu(maxlen=namespace.tabu_size)

    if namespace.max_distance and namespace.min_distance:
        message = "--max-distance and --min-distance are mutually exclusive"
        raise ValueError(message)

    propagation_priority_key: Callable[[Set[d2d.D2DPathSolution], d2d.D2DPathSolution], float] = utils.zero
    propagation_priority: Optional[str] = None
    if namespace.max_distance:
        propagation_priority = "max-distance"

        def propagation_priority_key(pareto_set: Set[d2d.D2DPathSolution], candidate: d2d.D2DPathSolution, /) -> float:
            cost = candidate.cost()
            result = 0.0
            for s in pareto_set:
                s_cost = s.cost()
                result += abs(s_cost[0] - cost[0]) + abs(s_cost[1] - cost[1])

            return -result

    if namespace.min_distance:
        propagation_priority = "min-distance"

        def propagation_priority_key(pareto_set: Set[d2d.D2DPathSolution], candidate: d2d.D2DPathSolution, /) -> float:
            cost = candidate.cost()
            result = 0.0
            for s in pareto_set:
                s_cost = s.cost()
                result += abs(s_cost[0] - cost[0]) + abs(s_cost[1] - cost[1])

            return result

    if namespace.profile:
        eval_func = f"""d2d.D2DPathSolution.tabu_search(
            pool_size={namespace.pool_size},
            iterations_count={namespace.iterations},
            use_tqdm={namespace.verbose},
            propagation_priority_key=propagation_priority_key,
            max_propagation={namespace.max_propagation},
            plot_pareto_front={namespace.verbose},
        )"""
        cProfile.run(eval_func)
        exit(0)

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

    print(f"Found {len(solutions)} solution(s):")
    for index, solution in enumerate(solutions):
        print(f"SOLUTION #{index + 1}: cost = {solution.cost()}")
        print("\n".join(f"Drone #{drone_index + 1}: {paths}" for drone_index, paths in enumerate(solution.drone_paths)))
        print("\n".join(f"Technician #{technician_index + 1}: {path}" for technician_index, path in enumerate(solution.technician_paths)))

        check = d2d.D2DPathSolution(
            drone_paths=solution.drone_paths,
            technician_paths=solution.technician_paths,
            drone_config_mapping=solution.drone_config_mapping,
        )
        assert check.cost() == solution.cost()

    if namespace.dump is not None:
        with open(namespace.dump, "w") as f:
            data = {
                "problem": namespace.problem,
                "iterations": namespace.iterations,
                "tabu-size": namespace.tabu_size,
                "energy-mode": namespace.energy_mode,
                "propagation-priority": propagation_priority,
                "solutions": [to_json(s) for s in solutions],
            }
            json.dump(data, f)

        print(f"Saved solution to {namespace.dump!r}")
