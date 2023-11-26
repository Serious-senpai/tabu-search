import argparse
import json
import os
import pathlib
import subprocess
import sys
from typing import Dict, List, Literal, Tuple, TYPE_CHECKING

from ts import utils


# Energy modes
LINEAR = "linear"
NON_LINEAR = "non-linear"


# Propagation priority
NONE = "none"
MIN_DISTANCE = "min-distance"
MAX_DISTANCE = "max-distance"
IDEAL_DISTANCE = "ideal-distance"


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: str
        iterations: int
        tabu_size: int
        drone_config_mapping: List[int]
        energy_mode: Literal["linear", "non-linear"]
        propagation_priority: Literal["none", "min-distance", "max-distance", "ideal-distance"]
        max_propagation: int
        verbose: bool
        pool_size: int
        repos: List[str]


prefix, *args = sys.argv[1:]
if prefix == "/d2d-compare":
    print(f"Running {__file__}")
    parser = argparse.ArgumentParser(description="Compare D2D solutions from multiple branches")
    parser.add_argument("problem", type=str, help="the problem name (e.g. \"6.5.1\", \"200.10.1\", ...)")
    parser.add_argument("-i", "--iterations", default=500, type=int, help="the number of iterations to run the tabu search for (default: 500)")
    parser.add_argument("-t", "--tabu-size", default=10, type=int, help="the tabu size for every neighborhood (default: 10)")
    parser.add_argument("-c", "--drone-config-mapping", nargs="+", default=[0, 0, 0, 0], type=int, help="the energy configuration index for each drone (default: \"0 0 0 0\")")
    parser.add_argument("-e", "--energy-mode", default=LINEAR, choices=[LINEAR, NON_LINEAR], help="the energy consumption mode to use (default: linear)")
    parser.add_argument("-k", "--propagation-priority", default=NONE, choices=[NONE, MIN_DISTANCE, MAX_DISTANCE, IDEAL_DISTANCE], help="set the solution propagation priority null (default: none)")
    parser.add_argument("-m", "--max-propagation", default=5, type=int, help="maximum number of propagating solutions at a time (default: 5)")
    parser.add_argument("-v", "--verbose", action="store_true", help="whether to display the progress bar and plot the solution")

    default_pool_size = os.cpu_count() or 1
    parser.add_argument("--pool-size", default=default_pool_size, type=int, help=f"the size of the process pool (default: {default_pool_size})")
    parser.add_argument("--repos", nargs="+", default=["."], type=str, help="the paths to the repositories")

    namespace = Namespace()
    parser.parse_args(args, namespace=namespace)
    print(namespace)

    pareto_fronts: Dict[str, List[Tuple[float, float]]] = {}
    for index, dir in enumerate(namespace.repos):
        output_file = pathlib.Path(f"d2d-summary/compare-index-{index}.json").absolute()
        args = [
            sys.executable, "d2d.py",
            namespace.problem,  # Feel free to do shell injection here, not my machine anyway
            "--iterations", str(namespace.iterations),
            "--tabu-size", str(namespace.tabu_size),
            "--drone-config-mapping", *map(str, namespace.drone_config_mapping),
            "--energy-mode", namespace.energy_mode,
            "--propagation-priority", namespace.propagation_priority,
            "--max-propagation", str(namespace.max_propagation),
            "--pool-size", str(namespace.pool_size),
            "--dump", str(output_file),
        ]
        if namespace.verbose:
            args.append("--verbose")

        process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=sys.stderr, shell=True, cwd=dir)
        process.wait()

        with open(output_file, "r") as file:
            data = json.load(file)

        pareto_fronts[dir] = []
        for d in data["solutions"]:
            pareto_fronts[dir].append(d["cost"])

    utils.plot_multi_fronts(
        [(front, dir) for dir, front in pareto_fronts.items()],
        dump=f"d2d-summary/compare.png",
        xlabel="Service duration (s)",
        ylabel="Total waiting time (s)",
    )
