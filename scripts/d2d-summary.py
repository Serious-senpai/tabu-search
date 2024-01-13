from __future__ import annotations

import argparse
import itertools
import json
import os
from collections import defaultdict
from datetime import timedelta
from pathlib import Path
from typing import DefaultDict, List, Literal, Optional, Tuple, TypedDict, TYPE_CHECKING

from tqdm import tqdm

from ts import utils


Choices = Literal[0, 1, 2]
choices = [0, 1, 2]


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        iterations: Choices
        tabu_size: Choices
        drone_config: Choices
        energy_mode: Choices
        propagation_priority: Choices
        extra: Choices


class SolutionJSON(TypedDict):
    cost: List[float]
    drone_paths: List[List[List[int]]]
    technician_paths: List[List[int]]


class ParetoFrontJSON(TypedDict):
    problem: str
    iterations: int
    tabu_size: int
    drone_config: int
    energy_mode: Literal["linear", "non-linear", "endurance"]
    propagation_priority: Literal[
        "none",
        "min-distance",
        "max-distance",
        "ideal-distance",
        "min-distance-no-normalize",
        "max-distance-no-normalize",
        "ideal-distance-no-normalize",
    ]
    solutions: List[SolutionJSON]
    extra: Optional[str]
    last_improved: int
    time: float


parser = argparse.ArgumentParser(
    description="Summarize D2D problem solutions and plot their Pareto fronts. Meaning of the following options: 0 - Do not include in plot, 1 - Include in plot file name, 2 - Include in plot for comparison",
)
parser.add_argument("--iterations", default=0, type=int, choices=choices)
parser.add_argument("--tabu-size", default=0, type=int, choices=choices)
parser.add_argument("--drone-config", default=0, type=int, choices=choices)
parser.add_argument("--energy-mode", default=0, type=int, choices=choices)
parser.add_argument("--propagation-priority", default=0, type=int, choices=choices)
parser.add_argument("--extra", default=0, type=int, choices=choices)


namespace = Namespace()
parser.parse_args(namespace=namespace)
print(namespace)


summary_dir = Path("d2d-summary/")


# Attributes of a problem: problem, drone configuration, energy mode
fronts: DefaultDict[str, DefaultDict[int, DefaultDict[str, List[Tuple[int, List[Tuple[float, float]]]]]]] = defaultdict(
    lambda: defaultdict(lambda: defaultdict(list)),
)
files = list(enumerate(sorted(os.listdir(summary_dir))))
for index, file in files:
    if file.startswith("output-"):
        with open(summary_dir / file, "r") as f:
            data: ParetoFrontJSON = json.load(f)

        front = [(d["cost"][0], d["cost"][1]) for d in data["solutions"]]
        fronts[data["problem"]][data["drone_config"]][data["energy_mode"]].append((index, front))


hv: List[Optional[float]] = [None] * len(files)
igd: List[Optional[float]] = [None] * len(files)
for problem in fronts.keys():
    for drone_config in fronts[problem].keys():
        for energy_mode in fronts[problem][drone_config].keys():
            pareto_fronts = fronts[problem][drone_config][energy_mode]

            hv_ref_point = (
                max(max(cost[0] for cost in front) for _, front in pareto_fronts),
                max(max(cost[1] for cost in front) for _, front in pareto_fronts),
            )
            igd_ref_front = tuple(utils.build_pareto_front(itertools.chain(*[front for _, front in pareto_fronts])))
            for index, front in pareto_fronts:
                hv[index] = utils.hypervolume(front, ref_point=hv_ref_point)
                igd[index] = utils.inverted_generational_distance(front, ref_costs=igd_ref_front)


print("Calculated HV and IGD")


def wrap_double_quotes(text: str) -> str:
    return f"\"{text}\""


field_names = (
    "ID",
    "Problem",
    "Iterations",
    "Tabu size",
    "Drone configuration",
    "Energy mode",
    "Propagation priority",
    "Solutions count",
    "Solutions",
    "Hypervolume",
    "Inverted generational distance",
    "Last improved",
    "Execution time",
)
with open(summary_dir / "d2d-summary.csv", "w") as csv:
    csv.write(",".join(field_names) + "\n")

    plot_fronts: DefaultDict[str, List[Tuple[List[Tuple[float, float]], str]]] = defaultdict(list)
    for index, file in tqdm(files, desc="Summarize to CSV", ascii=" █"):
        if file.startswith("output-"):
            with open(summary_dir / file, "r") as f:
                data = json.load(f)

            csv.write(
                ",".join(
                    (
                        str(index),
                        data["problem"],
                        str(data["iterations"]),
                        str(data["tabu_size"]),
                        str(data["drone_config"]),
                        data["energy_mode"],
                        data["propagation_priority"],
                        str(len(data["solutions"])),
                        wrap_double_quotes(", ".join(str(tuple(d["cost"])) for d in data["solutions"])),
                        str(hv[index]),
                        str(igd[index]),
                        str(data["last_improved"]),
                        str(timedelta(seconds=data["time"])),
                    )
                ) + "\n",
            )

            plot_front: List[Tuple[float, float]] = []
            for solution in data["solutions"]:
                plot_front.append((solution["cost"][0], solution["cost"][1]))

            plot_name = [data["problem"]]

            if namespace.iterations == 1:
                plot_name.append(str(data["iterations"]))

            if namespace.tabu_size == 1:
                plot_name.append(str(data["tabu_size"]))

            if namespace.drone_config == 1:
                plot_name.append(str(data["drone_config"]))

            if namespace.energy_mode == 1:
                plot_name.append(str(data["energy_mode"]))

            if namespace.propagation_priority == 1:
                plot_name.append(str(data["propagation_priority"]))

            if namespace.extra == 1:
                plot_name.append(str(data["extra"]))

            description: List[str] = []

            if namespace.iterations == 2:
                description.append(str(data["iterations"]))

            if namespace.tabu_size == 2:
                description.append(str(data["tabu_size"]))

            if namespace.drone_config == 2:
                description.append(str(data["drone_config"]))

            if namespace.energy_mode == 2:
                description.append(str(data["energy_mode"]))

            if namespace.propagation_priority == 2:
                description.append(str(data["propagation_priority"]))

            if namespace.extra == 2:
                description.append(str(data["extra"]))

            plot_fronts["-".join(plot_name)].append((plot_front, "-".join(description)))


for plot_filename, fronts_collection in tqdm(plot_fronts.items(), desc="Save figures to PNG", ascii=" █"):
    utils.plot_multi_fronts(
        fronts_collection,
        dump=f"d2d-summary/{plot_filename}.png",
        xlabel="Service duration (s)",
        ylabel="Total waiting time (s)",
    )
