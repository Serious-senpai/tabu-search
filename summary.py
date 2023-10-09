import csv
import json
import os
import re
import sys
from pathlib import Path
from typing import Dict


summary_dir = Path("summary/")
output = sys.argv[1]
field_names = ("Problem", "Iterations", "Tabu size", "Shuffle after", "Cost", "Path")
filename_pattern = re.compile(r"output-([a-z0-9]+)-(\d+)-(\d+)-(\d+)\.json")


def to_map(*args: str) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for index, key in enumerate(field_names):
        result[key] = args[index]

    return result


with open(output, "w", newline="") as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=field_names)
    writer.writeheader()

    for file in sorted(os.listdir(summary_dir)):
        if match := filename_pattern.fullmatch(file):
            problem, iterations, tabu_size, shuffle_after = match.groups()

            with open(summary_dir / file, "r") as f:
                data = json.load(f)

            assert problem == data["problem"]
            assert iterations == str(data["iterations"])
            assert tabu_size == str(data["tabu-size"])
            assert shuffle_after == str(data["shuffle-after"])

            writer.writerow(to_map(problem, iterations, tabu_size, shuffle_after, str(data["cost"]), str(data["path"])))
