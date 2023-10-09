from __future__ import annotations

import argparse
import cProfile
import json
from typing import Optional, TYPE_CHECKING

from tsp import PathSolution, Swap, SegmentShift, SegmentReverse


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: str
        iterations: int
        shuffle_after: int
        tabu_size: int
        profile: bool
        optimal: bool
        verbose: bool
        dump: Optional[str]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tabu search algorithm for TSP problems")
    parser.add_argument("problem", type=str, help="the problem name (e.g. \"a280\", \"berlin52\", ...)")
    parser.add_argument("-i", "--iterations", default=500, type=int, help="the number of iterations to run the tabu search for (default: 500)")
    parser.add_argument("-s", "--shuffle-after", default=10, type=int, help="after the specified number of non-improved iterations, shuffle the solution (default: 10)")
    parser.add_argument("-t", "--tabu-size", default=10, type=int, help="the tabu size for every neighborhood (default: 10)")
    parser.add_argument("-p", "--profile", action="store_true", help="run in profile mode and exit immediately")
    parser.add_argument("-o", "--optimal", action="store_true", help="read the optimal solution from the problem archive")
    parser.add_argument("-v", "--verbose", action="store_true", help="whether to display the progress bar and plot the solution")
    parser.add_argument("-d", "--dump", type=str, help="dump the solution to a file")

    namespace: Namespace = parser.parse_args()  # type: ignore
    print(namespace)
    PathSolution.import_problem(namespace.problem)

    if namespace.optimal:
        print("Reading optimal solution from the archive")
        if namespace.profile:
            cProfile.run("PathSolution.read_optimal_solution()")
            exit(0)
        else:
            solution = PathSolution.read_optimal_solution()

    else:
        print(f"Set all tabu size to {namespace.tabu_size}")
        Swap.reset_tabu(maxlen=namespace.tabu_size)
        SegmentShift.reset_tabu(maxlen=namespace.tabu_size)
        SegmentReverse.reset_tabu(maxlen=namespace.tabu_size)

        eval_func = f"PathSolution.tabu_search(iterations_count={namespace.iterations}, use_tqdm={namespace.verbose}, shuffle_after={namespace.shuffle_after})"
        print(f"Running {eval_func}")
        if namespace.profile:
            cProfile.run(eval_func)
            exit(0)
        else:
            solution = PathSolution.tabu_search(
                iterations_count=namespace.iterations,
                use_tqdm=namespace.verbose,
                shuffle_after=namespace.shuffle_after,
            )

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
