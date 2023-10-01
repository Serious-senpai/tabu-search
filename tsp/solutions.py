from __future__ import annotations

import re
from math import sqrt
from os import path
from typing import ClassVar, Tuple, TYPE_CHECKING

from .abc import BaseSolution
from .errors import ProblemNotFound, ProblemParsingException, UnsupportedEdgeWeightType


__all__ = (
    "PathSolution",
)


class PathSolution(BaseSolution):

    __slots__ = ()
    if TYPE_CHECKING:
        problem_name: ClassVar[str]
        dimension: ClassVar[int]
        edge_weight_type: ClassVar[str]
        distances: ClassVar[Tuple[Tuple[float, ...], ...]]

    @classmethod
    def import_problem(cls, problem: str, /) -> None:
        archive_file = path.join("problems", f"{problem}.tsp", f"{problem}.tsp")
        if not path.isfile(archive_file):
            raise ProblemNotFound(problem)

        cls.problem_name = problem
        try:
            with open(archive_file, "r") as file:
                data = file.read()

            cls.dimension = int(re.search(r"DIMENSION\s*:\s*(\d+)", data).group(1))
            cls.edge_weight_type = re.search(r"EDGE_WEIGHT_TYPE\s*:\s*(\w+)", data).group(1)

            if cls.edge_weight_type == "EUC_2D":
                distances = [[0.0] * cls.dimension for _ in range(cls.dimension)]
                x = []
                y = []
                for match in re.finditer(r"^\s*\d+\s+([\d\.\-+e]+\s+[\d\.\-+e]+)\s*?$", data, flags=re.MULTILINE):
                    _x, _y = map(float, match.group(1).split())
                    x.append(_x)
                    y.append(_y)

                for i in range(cls.dimension):
                    for j in range(i + 1, cls.dimension):
                        distances[i][j] = distances[j][i] = sqrt((x[i] - x[j]) ** 2 + (y[i] - y[j]) ** 2)

                cls.distances = tuple(tuple(row) for row in distances)

            else:
                raise UnsupportedEdgeWeightType(cls.edge_weight_type)

        except Exception as exc:
            raise ProblemParsingException(problem, exc) from exc
