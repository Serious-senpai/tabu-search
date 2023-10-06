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
        verbose: bool


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tabu search algorithm for TSP problems")
    parser.add_argument("problem", type=str, help="The problem name (e.g. \"a280\", \"berlin52\", ...)")
    parser.add_argument("-i", "--iter", type=int, required=True, help="The number of iterations to run the tabu search for")
    parser.add_argument("-s", "--tabu-size", type=int, required=True, help="The tabu size for every neighborhood")
    parser.add_argument("-v", "--verbose", action="store_true", help="Whether to display the progress bar")

    namespace: Namespace = parser.parse_args()  # type: ignore
    PathSolution.import_problem(namespace.problem)
    Swap.reset_tabu(maxlen=namespace.tabu_size)
    SegmentShift.reset_tabu(maxlen=namespace.tabu_size)
    SegmentReverse.reset_tabu(maxlen=namespace.tabu_size)

    print(f"Running tabu search with {namespace.iter} iterations.")
    solution = PathSolution.tabu_search(iterations_count=namespace.iter, use_tqdm=namespace.verbose)
    print(f"Solution cost = {solution.cost()}\nSolution path: {solution.get_path()}")

    if namespace.verbose:
        solution.plot()
