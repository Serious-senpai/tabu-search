import json
import os
import re
from io import TextIOWrapper
from pathlib import Path
from typing import Dict

from tsp import OptimalSolutionNotFound, PathSolution


summary_dir = Path("summary/")
field_names = ("Problem", "Iterations", "Tabu size", "Shuffle after", "Cost", "Path")
non_euc_pattern = re.compile(r"output-([a-z0-9]+)-(\d+)-(\d+)-(\d+)\.json")
euc_pattern = re.compile(r"output-([a-z0-9]+)-(\d+)-(\d+)-(\d+)-e\.json")


def to_map(*args: str) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for index, key in enumerate(field_names):
        result[key] = args[index]

    return result


def write_row(
    *,
    file: TextIOWrapper,
    path: Path,
    problem: str,
    iterations: str,
    tabu_size: str,
    shuffle_after: str,
) -> None:
    with open(path, "r") as f:
        data = json.load(f)

    assert problem == data["problem"]
    assert iterations == str(data["iterations"])
    assert tabu_size == str(data["tabu-size"])
    assert shuffle_after == str(data["shuffle-after"])

    file.write(",".join((problem, iterations, tabu_size, shuffle_after, str(data["cost"]), "\"" + str(data["path"]) + "\"")) + "\n")


def write_summary_rows(
    *,
    file: TextIOWrapper,
    problem: str,
) -> None:
    PathSolution.import_problem(problem)
    file.write("Minimum\nAverage\nOptimal")
    try:
        solution = PathSolution.read_optimal_solution()
        file.write(f",,,,{solution.cost()}")
    except OptimalSolutionNotFound:
        pass

    file.write("\n")


with open(summary_dir / "non-euclidean.csv", "w") as csv:
    csv.write(",".join(field_names) + "\n")
    for file in sorted(os.listdir(summary_dir)):
        if match := non_euc_pattern.fullmatch(file):
            problem, iterations, tabu_size, shuffle_after = match.groups()
            write_row(file=csv, path=summary_dir / file, problem=problem, iterations=iterations, tabu_size=tabu_size, shuffle_after=shuffle_after)


with open(summary_dir / "euclidean.csv", "w") as csv:
    csv.write(",".join(field_names) + "\n")

    last_problem = None
    for file in sorted(os.listdir(summary_dir)):
        if match := euc_pattern.fullmatch(file):
            problem, iterations, tabu_size, shuffle_after = match.groups()
            if last_problem is not None and problem != last_problem:
                write_summary_rows(file=csv, problem=last_problem)

            write_row(file=csv, path=summary_dir / file, problem=problem, iterations=iterations, tabu_size=tabu_size, shuffle_after=shuffle_after)
            last_problem = problem

    if problem is not None:
        write_summary_rows(file=csv, problem=last_problem)
