from __future__ import annotations

import os

from ..errors import ProblemParsingException, UnsupportedEdgeWeightType
from ..solutions import PathSolution


def test_import() -> None:
    for file in os.listdir("problems"):
        if file.endswith(".tsp"):
            problem = file.removesuffix(".tsp")
            print(f"Parsing {problem!r}")

            try:
                PathSolution.import_problem(problem)

            except ProblemParsingException as e:
                if isinstance(e.original, UnsupportedEdgeWeightType):
                    print(f"Unsupported edge_weight_type {e.original.edge_weight_type!r}")
                else:
                    raise

            else:
                print(f"Successfully parsed problem {problem!r}")
