from __future__ import annotations

import itertools
import os
from collections import deque
from multiprocessing import pool
from typing import ClassVar, Deque, List, Optional, Tuple, Set, TYPE_CHECKING

from .base import BasePathNeighborhood
from .bundle import IPCBundle
if TYPE_CHECKING:
    from ..solutions import PathSolution


__all__ = ("Swap",)


class Swap(BasePathNeighborhood[Tuple[int, int]]):

    __slots__ = ()
    _maxlen: ClassVar[int] = 100
    _tabu_list: ClassVar[Deque[Tuple[int, int]]] = deque()
    _tabu_set: ClassVar[Set[Tuple[int, int]]] = set()

    def swap(self, x: int, y: int) -> PathSolution:
        solution = self._solution

        before = list(solution.before)
        after = list(solution.after)

        if after[y] == x:
            x, y = y, x

        before_x = before[x]
        before_y = before[y]
        after_x = after[x]
        after_y = after[y]

        if after_x == y:
            cost = (
                solution.cost()
                + solution.distances[before_x][y] + solution.distances[x][after_y]
                - solution.distances[before_x][x] - solution.distances[y][after_y]
            )

            after[before_x], before[y] = y, before_x
            after[y], before[x] = x, y
            after[x], before[after_y] = after_y, x

        else:
            cost = (
                solution.cost()
                + solution.distances[before_x][y] + solution.distances[y][after_x]
                + solution.distances[before_y][x] + solution.distances[x][after_y]
                - solution.distances[before_x][x] - solution.distances[x][after_x]
                - solution.distances[before_y][y] - solution.distances[y][after_y]
            )

            before[x], before[y] = before_y, before_x
            after[x], after[y] = after_y, after_x

            after[before_x] = before[after_x] = y
            after[before_y] = before[after_y] = x

        return self.cls(after=after, before=before, cost=cost)

    def find_best_candidate(self, *, pool: pool.Pool) -> Optional[PathSolution]:
        concurrency = os.cpu_count() or 1
        args: List[IPCBundle[Swap, List[Tuple[int, int]]]] = [IPCBundle(self, []) for _ in range(concurrency)]
        for first, second in itertools.combinations(range(self._solution.dimension), 2):
            # first < second due to itertools.combinations implementation
            args[(first + second) % concurrency].data.append((first, second))

        result: Optional[PathSolution] = None
        min_pair: Optional[Tuple[int, int]] = None
        for result_temp, min_pair_temp in pool.map(self.static_find_best_candidate, args):
            if result_temp is None or min_pair_temp is None:
                continue

            if result is None or result_temp < result:
                result = result_temp
                min_pair = min_pair_temp

        if min_pair is not None:
            self.add_to_tabu(min_pair)

        return result

    @staticmethod
    def static_find_best_candidate(bundle: IPCBundle[Swap, List[Tuple[int, int]]]) -> Tuple[Optional[PathSolution], Optional[Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood._ensure_imported_data()

        result: Optional[PathSolution] = None
        min_pair: Optional[Tuple[int, int]] = None
        for pair in bundle.data:
            swapped = neighborhood.swap(*pair)
            if result is None or swapped < result:
                result = swapped
                min_pair = pair

        return result, min_pair
