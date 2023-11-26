import json
import sys
from typing import Dict, List, Tuple

from ts import utils


pareto_fronts: Dict[str, List[Tuple[float, float]]] = {}
for file in sys.argv[1:]:
    with open(file, "r") as f:
        data = json.load(f)

    pareto_fronts[file] = []
    for d in data["solutions"]:
        pareto_fronts[file].append(d["cost"])


utils.plot_multi_fronts(
    [(front, file) for file, front in pareto_fronts.items()],
    dump="compare.png",
    xlabel="Service duration (s)",
    ylabel="Total waiting time (s)",
)
