from __future__ import annotations

import itertools
import re
from collections import deque
from math import sqrt
from os import path
from typing import ClassVar, Deque, Iterable, Optional, Set, Tuple, TYPE_CHECKING

from matplotlib import axes, pyplot

from .abc import BaseNeighborhood, BaseSolution
from .errors import ProblemNotFound, ProblemParsingException, UnsupportedEdgeWeightType


__all__ = (
    "PathSolution",
    "SwapNeighborhood",
)


class PathSolution(BaseSolution):

    __slots__ = (
        "__cost",
        "after",
        "before",
    )
    if TYPE_CHECKING:
        __cost: Optional[float]
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

        self.__cost = cost

    def cost(self) -> float:
        if self.__cost is not None:
            return self.__cost

        result = 0.0
        last, current = 0, self.after[0]
        while current != 0:
            result += self.distances[last][current]
            last, current = current, self.after[current]

        result += self.distances[last][current]
        self.__cost = result
        return result

    def get_neighborhoods(self) -> Iterable[BaseNeighborhood[PathSolution]]:
        return [SwapNeighborhood(self)]

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
        for index in range(self.dimension):
            ax.annotate(index, (self.x[index], self.y[index]))

        ax.grid(True)

        pyplot.legend()
        pyplot.show()

    @classmethod
    def initial(cls) -> PathSolution:
        after = [-1] * cls.dimension
        before = [-1] * cls.dimension

        for index in range(cls.dimension):
            after[index] = (index + 1) % cls.dimension
            before[index] = (index - 1 + cls.dimension) % cls.dimension

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


class SwapNeighborhood(BaseNeighborhood[PathSolution]):

    __slots__ = ()
    __maxlen: ClassVar[int] = 100
    __tabu_list: ClassVar[Deque[Tuple[int, int]]] = deque()
    __tabu_set: ClassVar[Set[Tuple[int, int]]] = set()

    def swap(self, x: int, y: int) -> PathSolution:
        solution = self._solution

        before = list(solution.before)
        after = list(solution.after)

        before_x = before[x]
        before_y = before[y]
        after_x = after[x]
        after_y = after[y]

        cost = solution.cost()
        cost += (
            solution.distances[before_x][y] + solution.distances[y][after_x]
            + solution.distances[before_y][x] + solution.distances[x][after_y]
            - solution.distances[before_x][x] - solution.distances[x][after_x]
            - solution.distances[before_y][y] - solution.distances[y][after_y]
        )

        before[x], before[y] = before_y, before_x
        after[x], after[y] = after_y, after_x

        return PathSolution(before, after, cost=cost)

    def generate(self) -> Iterable[PathSolution]:
        for first, second in itertools.combinations(range(self._solution.dimension), 2):
            if self._solution.after[first] == second or self._solution.after[second] == first:
                continue

            pair = (first, second)
            if pair not in self.__tabu_set:
                yield self.swap(first, second)
                self._add_to_tabu(pair)

    @classmethod
    def _add_to_tabu(cls, pair: Tuple[int, int]) -> None:
        cls.__tabu_set.add(pair)
        cls.__tabu_list.append(pair)
        cls._remove_from_tabu()

    @classmethod
    def _remove_from_tabu(cls) -> None:
        while len(cls.__tabu_set) > cls.__maxlen:
            try:
                cls.__tabu_set.remove(cls.__tabu_list.popleft())
            except KeyError:
                pass

    @classmethod
    def reset_tabu(cls, *, maxlen: int = 100) -> None:
        cls.__maxlen = maxlen
        cls._remove_from_tabu()
