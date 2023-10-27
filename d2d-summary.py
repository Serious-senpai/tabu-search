import json
import os
import re
from pathlib import Path
from typing import Dict


summary_dir = Path("d2d-summary/")
field_names = ("Problem", "Iterations", "Tabu size", "Shuffle after", "Service duration", "Total waiting time", "Drone paths", "Technician paths")
pattern = re.compile(r"output-([0-9\.]+)-(\d+)-(\d+)-(\d+)\.json")


def to_map(*args: str) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for index, key in enumerate(field_names):
        result[key] = args[index]

    return result


with open(summary_dir / "summary.csv", "w") as csv:
    csv.write(",".join(field_names) + "\n")

    for file in sorted(os.listdir(summary_dir)):
        if match := pattern.fullmatch(file):
            problem, iterations, tabu_size, shuffle_after = match.groups()
            with open(summary_dir / file, "r") as f:
                data = json.load(f)

            assert problem == data["problem"]
            assert iterations == str(data["iterations"])
            assert tabu_size == str(data["tabu-size"])
            assert shuffle_after == str(data["shuffle-after"])

            for d in data["solutions"]:
                csv.write(
                    ",".join(
                        (
                            problem,
                            iterations,
                            tabu_size,
                            shuffle_after,
                            str(d["cost"][0]),
                            str(d["cost"][1]),
                            "\"" + str(d["drone_paths"]) + "\"",
                            "\"" + str(d["technician_paths"]) + "\"",
                        )
                    ) + "\n"
                )
