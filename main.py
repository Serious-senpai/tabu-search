import sys

from tsp import PathSolution, SwapNeighborhood, SegmentShift, SegmentReverse


if __name__ == "__main__":
    problem = sys.argv[1]
    PathSolution.import_problem(problem)

    try:
        iteration_count = int(sys.argv[2])
    except IndexError:
        iteration_count = 500
    else:
        print(f"Running tabu search with {iteration_count} iterations.")

    try:
        tabu_size = int(sys.argv[3])
    except IndexError:
        pass
    else:
        SwapNeighborhood.reset_tabu(maxlen=tabu_size)
        SegmentShift.reset_tabu(maxlen=tabu_size)
        SegmentReverse.reset_tabu(maxlen=tabu_size)
        print(f"Tabu size was set to {tabu_size}.")

    try:
        use_tqdm = not sys.argv[4] == "silent"
    except IndexError:
        use_tqdm = True

    solution = PathSolution.tabu_search(iterations_count=iteration_count, use_tqdm=use_tqdm)
    print(f"Solution cost = {solution.cost()}\nSolution path: {solution.get_path()}")
    solution.plot()
