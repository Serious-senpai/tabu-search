from __future__ import annotations

import argparse
import itertools
import json
import os
import subprocess
import sys
from typing import List, Literal, Set, TYPE_CHECKING

from ts import d2d, utils


# Energy modes
LINEAR = "linear"
NON_LINEAR = "non-linear"


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: str
        iterations: int
        tabu_size: int
        drone_config: int
        energy_mode: Literal["linear", "non-linear"]
        max_propagation: int
        verbose: bool
        pool_size: int


parser = argparse.ArgumentParser(description="Compare different propagation priorities for the D2D problem")
parser.add_argument("problem", type=str, help="the problem name (e.g. \"6.5.1\", \"200.10.1\", ...)")
parser.add_argument("-i", "--iterations", default=2000, type=int, help="the number of iterations to run the tabu search for (default: 2000)")
parser.add_argument("-t", "--tabu-size", default=10, type=int, help="the tabu size for every neighborhood (default: 10)")
parser.add_argument("-c", "--drone-config", default=0, type=int, help="the energy configuration index for each drone (default: 0)")
parser.add_argument("-e", "--energy-mode", default=LINEAR, choices=[LINEAR, NON_LINEAR], help="the energy consumption mode to use (default: linear)")
parser.add_argument("-m", "--max-propagation", default=5, type=int, help="maximum number of propagating solutions at a time (default: 5)")
parser.add_argument("-v", "--verbose", action="store_true", help="whether to display the progress bar and plot the solution")

default_pool_size = os.cpu_count() or 1
parser.add_argument("--pool-size", default=default_pool_size, type=int, help=f"the size of the process pool (default: {default_pool_size})")


namespace = Namespace()
parser.parse_args(namespace=namespace)
print(namespace)


print("Launching subprocesses")
processes: List[subprocess.Popen[bytes]] = []
outputs: List[str] = []
for propagation_priority in (
    "none",
    "min-distance",
    "max-distance",
    "ideal-distance",
):
    output = f"d2d-summary/{namespace.problem}.{propagation_priority}.json"

    command = [
        sys.executable,
        "d2d.py", namespace.problem,
        "--iterations", str(namespace.iterations),
        "--tabu-size", str(namespace.tabu_size),
        "--drone-config", str(namespace.drone_config),
        "--energy-mode", namespace.energy_mode,
        "--propagation-priority", propagation_priority,
        "--dump", output,
        "--pool-size", str(namespace.pool_size),
    ]
    if namespace.verbose:
        command.append("--verbose")

    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    processes.append(process)
    outputs.append(output)


print(f"Launched {len(processes)} subprocesses:", ", ".join(str(process.pid) for process in processes))
has_exception = False
for process in processes:
    _, stderr = process.communicate()
    print(f"Process {process.pid} exited with code {process.returncode}.")
    if process.returncode != 0:
        has_exception = True
        print(stderr.decode("utf-8"))


print("Plotting Pareto fronts...")
subprocess.Popen(
    [
        sys.executable,
        "scripts/d2d-compare.py",
        *outputs,
    ],
    stdout=sys.stdout,
    stderr=sys.stderr,
).wait()


d2d.D2DPathSolution.import_problem(
    namespace.problem,
    drone_config=namespace.drone_config,
    energy_mode=namespace.energy_mode,
)


solutions: List[List[d2d.D2DPathSolution]] = []
for output in outputs:
    solutions.append([])
    with open(output, "r") as file:
        data = json.load(file)

    for s in data["solutions"]:
        solutions[-1].append(
            d2d.D2DPathSolution(
                drone_paths=tuple(tuple(tuple(path) for path in paths) for paths in s["drone_paths"]),
                technician_paths=tuple(tuple(path) for path in s["technician_paths"]),
            )
        )


merged: Set[d2d.D2DPathSolution] = set()
for solution in itertools.chain(*solutions):
    solution.add_to_pareto_set(merged)
merged_costs = [s.cost() for s in merged]
hv_reference_point = max(cost[0] for cost in merged_costs), max(cost[1] for cost in merged_costs)


for output, pareto_front in zip(outputs, solutions):
    costs = [s.cost() for s in pareto_front]
    hv = utils.hypervolume(costs, ref_point=hv_reference_point)
    igd = utils.inverted_generational_distance(costs, ref_costs=merged_costs)

    solution_display = utils.ngettext(len(pareto_front) == 1, "solution", "solutions")
    print(f"{output} ({len(pareto_front)} {solution_display}):: HV = {hv} IGD = {igd}")


if has_exception:
    print("Returning code 1 due to failure in some subprocesses")
    sys.exit(1)
