import sys

from tsp import PathSolution, SwapNeighborhood


if __name__ == "__main__":
    problem = sys.argv[1]
    PathSolution.import_problem(problem)

    try:
        iteration_count = int(sys.argv[2])
    except IndexError:
        iteration_count = 30
    else:
        print(f"Running tabu search with {iteration_count} iterations. Press Ctrl-C to terminate.")

    try:
        tabu_size = int(sys.argv[3])
    except IndexError:
        pass
    else:
        SwapNeighborhood.reset_tabu(maxlen=tabu_size)
        print(f"Tabu size was set to {tabu_size}.")

    solution = PathSolution.tabu_search(iterations_count=iteration_count)
    print(f"Solution cost = {solution.cost()}\nSolution path: {solution.get_path()}")
    solution.plot()
