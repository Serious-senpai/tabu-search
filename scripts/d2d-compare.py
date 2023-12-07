import json
import sys
import traceback
from pathlib import Path
from typing import Dict, List, Tuple

from ts import utils


pareto_fronts: Dict[str, List[Tuple[float, float]]] = {}
for file in sys.argv[1:]:
    try:
        with open(file, "r") as f:
            data = json.load(f)

    except FileNotFoundError:
        print(f"Ignoring non-existent file {file}")
        traceback.print_exc()

    else:
        pareto_fronts[file] = []
        for d in data["solutions"]:
            pareto_fronts[file].append(d["cost"])


utils.plot_multi_fronts(
    [(front, Path(file).stem) for file, front in pareto_fronts.items()],
    dump="compare.png",
    xlabel="Service duration (s)",
    ylabel="Total waiting time (s)",
)
