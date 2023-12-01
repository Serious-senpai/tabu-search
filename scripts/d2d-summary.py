import json
import os
from pathlib import Path
from typing import Dict, List, Tuple

from ts import utils


summary_dir = Path("d2d-summary/")
field_names = (
    "ID",
    "Problem",
    "Iterations",
    "Tabu size",
    "Energy mode",
    "Propagation priority",
    "Hypervolume",
    "Service duration (s)",
    "Total waiting time (s)",
    "Drone config mapping",
    "Drone paths",
    "Technician paths",
)


pareto_fronts: Dict[str, List[Tuple[List[Tuple[float, float]], str]]] = {}
with open(summary_dir / "d2d-summary.csv", "w") as csv:
    csv.write(",".join(field_names) + "\n")

    for index, file in enumerate(sorted(os.listdir(summary_dir))):
        if file.startswith("output-"):
            with open(summary_dir / file, "r") as f:
                data = json.load(f)

            front: List[Tuple[float, float]] = []
            for d in data["solutions"]:
                csv.write(
                    ",".join(
                        (
                            str(index),
                            data["problem"],
                            str(data["iterations"]),
                            str(data["tabu-size"]),
                            data["energy-mode"],
                            data["propagation-priority"],
                            str(data["hypervolume"]),
                            str(d["cost"][0]),
                            str(d["cost"][1]),
                            "\"" + str(d["drone_config_mapping"]) + "\"",
                            "\"" + str(d["drone_paths"]) + "\"",
                            "\"" + str(d["technician_paths"]) + "\"",
                        )
                    ) + "\n"
                )
                front.append(tuple(d["cost"]))  # type: ignore

            key = data["problem"] + "-" + data["energy-mode"]
            try:
                pareto_fronts[key].append((front, data["propagation-priority"]))
            except KeyError:
                pareto_fronts[key] = [(front, data["propagation-priority"])]


for key, fronts in pareto_fronts.items():
    utils.plot_multi_fronts(
        fronts,
        dump=f"d2d-summary/{key}.png",
        xlabel="Service duration (s)",
        ylabel="Total waiting time (s)",
    )
