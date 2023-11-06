import json
import os
from pathlib import Path
from typing import Dict


summary_dir = Path("d2d-summary/")
field_names = ("Problem", "Iterations", "Tabu size", "Energy mode", "Propagation priority", "Service duration", "Total waiting time", "Drone config mapping", "Drone paths", "Technician paths")


def to_map(*args: str) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for index, key in enumerate(field_names):
        result[key] = args[index]

    return result


with open(summary_dir / "d2d-summary.csv", "w") as csv:
    csv.write(",".join(field_names) + "\n")

    for file in sorted(os.listdir(summary_dir)):
        if file.startswith("output-"):
            with open(summary_dir / file, "r") as f:
                data = json.load(f)

            for d in data["solutions"]:
                csv.write(
                    ",".join(
                        (
                            data["problem"],
                            str(data["iterations"]),
                            str(data["tabu-size"]),
                            data["energy-mode"],
                            data["propagation-priority"] or "",
                            str(d["cost"][0]),
                            str(d["cost"][1]),
                            "\"" + str(d["drone_config_mapping"]) + "\"",
                            "\"" + str(d["drone_paths"]) + "\"",
                            "\"" + str(d["technician_paths"]) + "\"",
                        )
                    ) + "\n"
                )
