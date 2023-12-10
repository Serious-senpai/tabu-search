from __future__ import annotations

import itertools
import random
import re
from math import sqrt
from multiprocessing import pool as p
from os import path
from typing import Any, ClassVar, Final, List, Optional, Tuple, Union, TYPE_CHECKING

from matplotlib import axes, pyplot
from tqdm import tqdm

from .errors import OptimalSolutionNotFound, ProblemNotFound, ProblemParsingException, UnsupportedEdgeWeightType
from .neighborhoods import SegmentReverse, SegmentShift, Swap
from ..abc import SingleObjectiveNeighborhood, SingleObjectiveSolution


__all__ = (
    "TSPPathSolution",
)


class TSPPathSolution(SingleObjectiveSolution):
    """Represents a solution to the TSP problem"""

    __slots__ = (
        "_cost",
        "_path",
        "after",
        "before",
    )
    problem_name: ClassVar[Optional[str]] = None
    if TYPE_CHECKING:
        _cost: float
        _path: Optional[Tuple[int, ...]]
        after: Final[Tuple[int, ...]]
        before: Final[Tuple[int, ...]]

        dimension: ClassVar[int]
        edge_weight_type: ClassVar[str]
        distances: ClassVar[Tuple[Tuple[float, ...], ...]]

        x: Tuple[float, ...]
        y: Tuple[float, ...]

    def __init__(self, *, after: Tuple[int, ...], before: Tuple[int, ...], cost: Optional[float] = None) -> None:
        self.after = after
        self.before = before

        if cost is None:
            result = 0.0
            last, current = 0, self.after[0]
            while current != 0:
                result += self.distances[last][current]
                last, current = current, self.after[current]

            result += self.distances[last][current]
            self._cost = result

        else:
            self._cost = cost

        self._path = None

    @property
    def path(self) -> Tuple[int, ...]:
        if self._path is not None:
            return self._path

        path = [0]
        current = self.after[0]
        while current != 0:
            path.append(current)
            current = self.after[current]

        self._path = tuple(path)
        return self._path

    def cost(self) -> float:
        return self._cost

    def post_optimization(self, *, pool: p.Pool, pool_size: int, use_tqdm: bool) -> TSPPathSolution:
        result = self
        iterations: Union[Tuple[SingleObjectiveNeighborhood[TSPPathSolution, Any], ...], tqdm[SingleObjectiveNeighborhood[TSPPathSolution, Any]]] = self.get_neighborhoods()
        if use_tqdm:
            iterations = tqdm(iterations, desc="Post-optimization", ascii=" █", colour="blue")

        for neighborhood in iterations:
            candidate = neighborhood.find_best_candidate(pool=pool, pool_size=pool_size)
            if candidate is not None:
                result = min(result, candidate)

        return result

    def get_neighborhoods(self) -> Tuple[SingleObjectiveNeighborhood[TSPPathSolution, Any], ...]:
        return (
            Swap(self, first_length=1, second_length=1),
            Swap(self, first_length=2, second_length=1),
            Swap(self, first_length=2, second_length=2),
            Swap(self, first_length=3, second_length=1),
            Swap(self, first_length=3, second_length=2),
            SegmentShift(self, segment_length=1),
            SegmentShift(self, segment_length=2),
            SegmentShift(self, segment_length=3),
            SegmentReverse(self, segment_length=4),
            SegmentReverse(self, segment_length=5),
            SegmentReverse(self, segment_length=6),
        )

    def shuffle(self, *, use_tqdm: bool = True) -> TSPPathSolution:
        def adjacent_distance(index: int) -> float:
            return self.distances[index][self.after[index]] + self.distances[index][self.before[index]]

        indices = sorted(range(self.dimension), key=adjacent_distance, reverse=True)[:self.dimension // 2]
        iterations: Union[List[int], tqdm[int]] = indices
        if use_tqdm:
            iterations = tqdm(iterations, desc="Shuffle", ascii=" █", colour="red")

        result = self
        for index in iterations:
            other = random.choice(indices)
            if other == index:
                other = (other + 1) % self.dimension

            result = Swap(result, first_length=1, second_length=1).swap(index, index, other, other)

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
        for index in range(self.dimension):
            ax.annotate(str(index), (self.x[index], self.y[index]))

        ax.grid(True)

        pyplot.legend()
        pyplot.show()
        pyplot.close()

    @classmethod
    def initial(cls) -> TSPPathSolution:
        after = [-1] * cls.dimension
        before = [-1] * cls.dimension

        path = [0]
        cities = set(range(1, cls.dimension))
        while len(cities) > 0:
            current = path[-1]
            insert = min(cities, key=cls.distances[current].__getitem__)
            path.append(insert)
            cities.remove(insert)

        for index in range(cls.dimension):
            after[path[index]] = path[(index + 1) % cls.dimension]
            before[path[index]] = path[(index - 1 + cls.dimension) % cls.dimension]

        return cls(after=tuple(after), before=tuple(before))

    @classmethod
    def read_optimal_solution(cls) -> TSPPathSolution:
        archive_file = path.join("problems", "tsp", f"{cls.problem_name}.opt.tour", f"{cls.problem_name}.opt.tour")
        if not path.isfile(archive_file):
            raise OptimalSolutionNotFound(cls.problem_name)

        with open(archive_file, "r") as file:
            sol_path: List[int] = []
            parse_start = False
            for line in file.readlines():
                if parse_start:
                    index = int(line) - 1
                    if index >= 0:
                        sol_path.append(index)
                    else:
                        break

                if line.strip() == "TOUR_SECTION":
                    parse_start = True

        return cls.from_path(sol_path)

    @classmethod
    def from_path(cls, path: Union[List[int], Tuple[int, ...]], /) -> TSPPathSolution:
        after = [-1] * cls.dimension
        before = [-1] * cls.dimension
        for index in range(cls.dimension):
            current = path[index]
            after[current] = path[(index + 1) % cls.dimension]
            before[current] = path[(index + cls.dimension - 1) % cls.dimension]

        return cls(after=tuple(after), before=tuple(before))

    @classmethod
    def import_problem(cls, problem: str, *, precalculated_distances: Optional[Tuple[Tuple[float, ...], ...]] = None) -> None:
        problem = problem.removesuffix(".tsp")
        archive_file = path.join("problems", "tsp", f"{problem}.tsp", f"{problem}.tsp")
        if not path.isfile(archive_file):
            raise ProblemNotFound(problem)

        cls.problem_name = problem
        try:
            with open(archive_file, "r") as file:
                data = file.read()

            cls.dimension = int(re.search(r"DIMENSION\s*:\s*(\d+)", data).group(1))  # type: ignore
            cls.edge_weight_type = re.search(r"EDGE_WEIGHT_TYPE\s*:\s*(\w+)", data).group(1)  # type: ignore

            if cls.edge_weight_type == "EUC_2D":
                x = []
                y = []
                for match in re.finditer(r"^\s*\d+\s+([\d\.\-+e]+\s+[\d\.\-+e]+)\s*?$", data, flags=re.MULTILINE):
                    _x, _y = map(float, match.group(1).split())
                    x.append(_x)
                    y.append(_y)

                cls.x = tuple(x)
                cls.y = tuple(y)

                if precalculated_distances is None:
                    distances = [[0.0] * cls.dimension for _ in range(cls.dimension)]
                    for i, j in itertools.combinations(range(cls.dimension), 2):
                        distances[i][j] = distances[j][i] = int(sqrt((x[i] - x[j]) ** 2 + (y[i] - y[j]) ** 2))

                    cls.distances = tuple(tuple(row) for row in distances)

                else:
                    cls.distances = precalculated_distances

            else:
                raise UnsupportedEdgeWeightType(cls.edge_weight_type)

        except Exception as exc:
            raise ProblemParsingException(problem, exc) from exc

    def __hash__(self) -> int:
        return hash(self.after)
