from __future__ import annotations

import re
from math import sqrt
from os import path
from typing import ClassVar, Iterable, Optional, Tuple, TYPE_CHECKING

from matplotlib import axes, pyplot

from .abc import BaseSolution
from .errors import ProblemNotFound, ProblemParsingException, UnsupportedEdgeWeightType


__all__ = (
    "PathSolution",
)


class PathSolution(BaseSolution):

    __slots__ = (
        "__cost",
        "after",
        "before",
    )
    if TYPE_CHECKING:
        __cost: float
        after: Tuple[int, ...]
        before: Tuple[int, ...]

        problem_name: ClassVar[str]
        dimension: ClassVar[int]
        edge_weight_type: ClassVar[str]
        distances: ClassVar[Tuple[Tuple[float, ...], ...]]

        x: Tuple[float, ...]
        y: Tuple[float, ...]

    def __init__(self, before: Iterable[int], after: Iterable[int], *, cost: Optional[float] = None) -> None:
        self.after = tuple(after)
        self.before = tuple(before)

        if cost is not None:
            self.__cost = cost

    def cost(self) -> float:
        if self.__cost is not None:
            return self.__cost

        result = 0.0
        last, current = 0, self.after[0]
        while current != 0:
            result += self.distances[last][current]

        result += self.distances[last][current]
        self.__cost = result
        return result

    def plot(self) -> None:
        _, ax = pyplot.subplots()
        assert isinstance(ax, axes.Axes)

        quiver_kwargs = {
            "color": "darkblue",
            "angles": "xy",
            "scale_units": "xy",
            "scale": 1,
        }
        for index in range(self.dimension):
            next = self.after[index]
            ax.quiver(self.x[index], self.y[index], self.x[next] - self.x[index], self.y[next] - self.y[index], **quiver_kwargs)

        ax.scatter(self.x, self.y, c="blue", label="City")
        ax.grid(True)

        pyplot.legend()
        pyplot.show()

    @classmethod
    def initial(cls) -> PathSolution:
        after = [-1] * cls.dimension
        before = [-1] * cls.dimension

        for index in range(cls.dimension):
            after[index] = (index + cls.dimension - 1) % cls.dimension
            before[index] = (index + 1) % cls.dimension

        return cls(before, after)

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

                cls.x = tuple(x)
                cls.y = tuple(y)

                for i in range(cls.dimension):
                    for j in range(i + 1, cls.dimension):
                        distances[i][j] = distances[j][i] = sqrt((x[i] - x[j]) ** 2 + (y[i] - y[j]) ** 2)

                cls.distances = tuple(tuple(row) for row in distances)

            else:
                raise UnsupportedEdgeWeightType(cls.edge_weight_type)

        except Exception as exc:
            raise ProblemParsingException(problem, exc) from exc

    def __hash__(self) -> int:
        return hash(self.after)
