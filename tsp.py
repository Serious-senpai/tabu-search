from __future__ import annotations

import argparse
import json
import os
from typing import Optional, TYPE_CHECKING

from ts import tsp, utils


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: str
        iterations: int
        shuffle_after: int
        tabu_size: int
        optimal: bool
        verbose: bool
        dump: Optional[str]
        pool_size: int


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tabu search algorithm for TSP problems")
    parser.add_argument("problem", type=str, help="the problem name (e.g. \"a280\", \"berlin52\", ...)")
    parser.add_argument("-i", "--iterations", default=500, type=int, help="the number of iterations to run the tabu search for (default: 500)")
    parser.add_argument("-s", "--shuffle-after", default=50, type=int, help="after the specified number of non-improved iterations, shuffle the solution (default: 50)")
    parser.add_argument("-t", "--tabu-size", default=10, type=int, help="the tabu size for every neighborhood (default: 10)")
    parser.add_argument("-o", "--optimal", action="store_true", help="read the optimal solution from the problem archive")
    parser.add_argument("-v", "--verbose", action="store_true", help="whether to display the progress bar and plot the solution")
    parser.add_argument("-d", "--dump", type=str, help="dump the solution to a file")

    default_pool_size = os.cpu_count() or 1
    parser.add_argument("--pool-size", default=default_pool_size, type=int, help=f"the size of the process pool (default: {default_pool_size})")

    utils.display_platform()

    namespace = Namespace()
    parser.parse_args(namespace=namespace)
    print(namespace)
    tsp.TSPPathSolution.import_problem(namespace.problem)

    if namespace.optimal:
        print("Reading optimal solution from the archive")
        solution = tsp.TSPPathSolution.read_optimal_solution()

    else:
        tsp.Swap.reset_tabu(maxlen=namespace.tabu_size)
        tsp.SegmentShift.reset_tabu(maxlen=namespace.tabu_size)
        tsp.SegmentReverse.reset_tabu(maxlen=namespace.tabu_size)

        solution = tsp.TSPPathSolution.tabu_search(
            pool_size=namespace.pool_size,
            iterations_count=namespace.iterations,
            use_tqdm=namespace.verbose,
            shuffle_after=namespace.shuffle_after,
        )

        check = tsp.TSPPathSolution(after=solution.after, before=solution.before)
        assert check.cost() == solution.cost()

    print(f"Solution cost = {solution.cost()}\nSolution path: {solution.path}")

    if namespace.verbose:
        solution.plot()

    if namespace.dump is not None:
        with open(namespace.dump, "w") as f:
            data = {
                "problem": namespace.problem,
                "iterations": namespace.iterations,
                "tabu-size": namespace.tabu_size,
                "shuffle-after": namespace.shuffle_after,
                "cost": solution.cost(),
                "path": solution.path,
            }
            json.dump(data, f)

        print(f"Saved solution to {namespace.dump!r}")
