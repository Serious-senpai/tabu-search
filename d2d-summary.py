import json
import os
import re
from pathlib import Path
from typing import Dict


summary_dir = Path("d2d-summary/")
field_names = ("Problem", "Iterations", "Tabu size", "Propagation priority", "Service duration", "Total waiting time", "Drone paths", "Technician paths")
pattern = re.compile(r"output-([0-9\.]+)-(\d+)-(\d+)-([-a-z]+)?\.json")


def to_map(*args: str) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for index, key in enumerate(field_names):
        result[key] = args[index]

    return result


with open(summary_dir / "summary.csv", "w") as csv:
    csv.write(",".join(field_names) + "\n")

    for file in sorted(os.listdir(summary_dir)):
        if match := pattern.fullmatch(file):
            groups = match.groups()
            problem, iterations, tabu_size, propagation_priority = groups
            if propagation_priority is None:
                propagation_priority = ""

            with open(summary_dir / file, "r") as f:
                data = json.load(f)

            assert problem == data["problem"]
            assert iterations == str(data["iterations"])
            assert tabu_size == str(data["tabu-size"])

            for d in data["solutions"]:
                csv.write(
                    ",".join(
                        (
                            problem,
                            iterations,
                            tabu_size,
                            propagation_priority.strip("-"),
                            str(d["cost"][0]),
                            str(d["cost"][1]),
                            "\"" + str(d["drone_paths"]) + "\"",
                            "\"" + str(d["technician_paths"]) + "\"",
                        )
                    ) + "\n"
                )
