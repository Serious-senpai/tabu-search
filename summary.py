import csv
import json
import os
import re
from pathlib import Path
from typing import Dict


summary_dir = Path("summary/")
field_names = ("Problem", "Iterations", "Tabu size", "Shuffle after", "Cost", "Path")
non_euc_pattern = re.compile(r"output-([a-z0-9]+)-(\d+)-(\d+)-(\d+)\.json")
euc_pattern = re.compile(r"output-([a-z0-9]+)-(\d+)-(\d+)-(\d+)-e\.json")


def to_map(*args: str) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for index, key in enumerate(field_names):
        result[key] = args[index]

    return result


def write_to_csv(
    *,
    writer: csv.DictWriter,
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

    writer.writerow(to_map(problem, iterations, tabu_size, shuffle_after, str(data["cost"]), str(data["path"])))


with open(summary_dir / "non-euclidean.csv", "w", newline="") as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=field_names)
    writer.writeheader()

    for file in sorted(os.listdir(summary_dir)):
        if match := non_euc_pattern.fullmatch(file):
            problem, iterations, tabu_size, shuffle_after = match.groups()
            write_to_csv(writer=writer, path=summary_dir / file, problem=problem, iterations=iterations, tabu_size=tabu_size, shuffle_after=shuffle_after)


with open(summary_dir / "euclidean.csv", "w", newline="") as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=field_names)
    writer.writeheader()

    for file in sorted(os.listdir(summary_dir)):
        if match := euc_pattern.fullmatch(file):
            problem, iterations, tabu_size, shuffle_after = match.groups()
            write_to_csv(writer=writer, path=summary_dir / file, problem=problem, iterations=iterations, tabu_size=tabu_size, shuffle_after=shuffle_after)
