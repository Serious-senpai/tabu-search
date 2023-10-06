from __future__ import annotations

import random
import re
from os import path
from typing import ClassVar, Iterable, List, Optional, Tuple, Union, TYPE_CHECKING

from matplotlib import axes, pyplot
from tqdm import tqdm

from .abc import BaseNeighborhood, BaseSolution
from .errors import OptimalSolutionNotFound, ProblemNotFound, ProblemParsingException, UnsupportedEdgeWeightType
from .neighborhoods import SegmentReverse, SegmentShift, Swap


__all__ = (
    "PathSolution",
)


class PathSolution(BaseSolution):

    __slots__ = (
        "_cost",
        "after",
        "before",
    )
    problem_name: ClassVar[Optional[str]] = None
    if TYPE_CHECKING:
        _cost: Optional[float]
        after: Tuple[int, ...]
        before: Tuple[int, ...]

        dimension: ClassVar[int]
        edge_weight_type: ClassVar[str]
        distances: ClassVar[Tuple[Tuple[float, ...], ...]]

        x: Tuple[float, ...]
        y: Tuple[float, ...]

    def __init__(self, *, after: Iterable[int], before: Iterable[int], cost: Optional[float] = None) -> None:
        self.after = tuple(after)
        self.before = tuple(before)

        self._cost = cost

    def cost(self) -> float:
        if self._cost is not None:
            return self._cost

        result = 0.0
        last, current = 0, self.after[0]
        while current != 0:
            result += self.distances[last][current]
            last, current = current, self.after[current]

        result += self.distances[last][current]
        self._cost = result
        return result

    def get_neighborhoods(self) -> Tuple[BaseNeighborhood[PathSolution], ...]:
        return (
            Swap(self),
            SegmentShift(self, segment_length=1),
            SegmentShift(self, segment_length=2),
            SegmentShift(self, segment_length=3),
            SegmentShift(self, segment_length=4),
            SegmentShift(self, segment_length=5),
            SegmentShift(self, segment_length=6),
            SegmentShift(self, segment_length=7),
            SegmentReverse(self, segment_length=4),
            SegmentReverse(self, segment_length=5),
            SegmentReverse(self, segment_length=6),
            SegmentReverse(self, segment_length=7),
        )

    def shuffle(self, use_tqdm: bool = True) -> PathSolution:
        indices = list(range(self.dimension))
        random.shuffle(indices)
        result = self

        iterations: Union[List[int], tqdm[int]] = indices[:self.dimension // 3]
        if use_tqdm:
            iterations = tqdm(iterations, desc="Shuffle", ascii=" █", colour="red")

        for index in iterations:
            other = random.choice(indices)
            after = result.after.__getitem__
            while other == index or after(other) == index or after(after(other)) == index or after(index) == other or after(after(index)) == other:
                other = (other + 1) % self.dimension

            result = Swap(result).swap(index, other)

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

    def get_path(self) -> Tuple[int, ...]:
        path = [0]
        current = self.after[0]
        while current != 0:
            path.append(current)
            current = self.after[current]

        return tuple(path)

    @classmethod
    def initial(cls) -> PathSolution:
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

        return cls(after=after, before=before)

    @classmethod
    def read_optimal_solution(cls) -> PathSolution:
        archive_file = path.join("problems", f"{cls.problem_name}.opt.tour", f"{cls.problem_name}.opt.tour")
        if not path.isfile(archive_file):
            raise OptimalSolutionNotFound(cls.problem_name)

        with open(archive_file, "r") as file:
            spath: List[int] = []
            parse_start = False
            for line in file.readlines():
                if parse_start:
                    index = int(line) - 1
                    if index >= 0:
                        spath.append(index)
                    else:
                        break

                if line.strip() == "TOUR_SECTION":
                    parse_start = True

        after = [-1] * cls.dimension
        before = [-1] * cls.dimension
        for index in range(cls.dimension):
            current = spath[index]
            after[current] = spath[(index + 1) % cls.dimension]
            before[current] = spath[(index + cls.dimension - 1) % cls.dimension]

        return PathSolution(after=after, before=before)

    @classmethod
    def import_problem(cls, problem: str, *, precalculated_distances: Optional[Tuple[Tuple[float, ...], ...]] = None) -> None:
        archive_file = path.join("problems", f"{problem}.tsp", f"{problem}.tsp")
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
                    for i in range(cls.dimension):
                        for j in range(i + 1, cls.dimension):
                            distances[i][j] = distances[j][i] = abs(x[i] - x[j]) + abs(y[i] - y[j])

                    cls.distances = tuple(tuple(row) for row in distances)

                else:
                    cls.distances = precalculated_distances

            else:
                raise UnsupportedEdgeWeightType(cls.edge_weight_type)

        except Exception as exc:
            raise ProblemParsingException(problem, exc) from exc

    def __hash__(self) -> int:
        return hash(self.after)
