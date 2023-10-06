from __future__ import annotations

import argparse
import sys
from typing import TYPE_CHECKING

from tsp import PathSolution, Swap, SegmentShift, SegmentReverse


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: str
        iter: int
        tabu_size: int
        optimal: bool
        verbose: bool


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tabu search algorithm for TSP problems")
    parser.add_argument("problem", type=str, help="the problem name (e.g. \"a280\", \"berlin52\", ...)")
    parser.add_argument("-i", "--iter", default=500, type=int, help="the number of iterations to run the tabu search for (default: 500)")
    parser.add_argument("-s", "--tabu-size", default=10, type=int, help="the tabu size for every neighborhood (default: 10)")
    parser.add_argument("-o", "--optimal", action="store_true", help="read the optimal solution from the problem archive")
    parser.add_argument("-v", "--verbose", action="store_true", help="whether to display the progress bar and plot the solution")

    namespace: Namespace = parser.parse_args()  # type: ignore
    PathSolution.import_problem(namespace.problem)

    if namespace.optimal:
        print("Reading optimal solution from the archive")
        solution = PathSolution.read_optimal_solution()

    else:
        Swap.reset_tabu(maxlen=namespace.tabu_size)
        SegmentShift.reset_tabu(maxlen=namespace.tabu_size)
        SegmentReverse.reset_tabu(maxlen=namespace.tabu_size)

        print(f"Running tabu search with {namespace.iter} iterations.")
        solution = PathSolution.tabu_search(iterations_count=namespace.iter, use_tqdm=namespace.verbose)

    print(f"Solution cost = {solution.cost()}\nSolution path: {solution.get_path()}")

    if namespace.verbose:
        solution.plot()
