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


__all__ = ("SegmentReverse",)


class SegmentReverse(BasePathNeighborhood[Tuple[int, int]]):

    __slots__ = (
        "_segment_length",
    )
    _maxlen: ClassVar[int] = 100
    _tabu_list: ClassVar[Deque[Tuple[int, int]]] = deque()
    _tabu_set: ClassVar[Set[Tuple[int, int]]] = set()
    if TYPE_CHECKING:
        _segment_length: int

    def __init__(self, solution: PathSolution, *, segment_length: int) -> None:
        super().__init__(solution)
        self._segment_length = segment_length
        if segment_length < 3:
            raise ValueError("Segment length must be 3 or more")

    def reverse(self, segment: List[int]) -> PathSolution:
        solution = self._solution

        before = list(solution.before)
        after = list(solution.after)

        before_segment = before[segment[0]]
        after_segment = after[segment[-1]]

        cost = (
            solution.cost()
            + solution.distances[before_segment][segment[-1]] + solution.distances[segment[0]][after_segment]
            - solution.distances[before_segment][segment[0]] - solution.distances[segment[-1]][after_segment]
        )

        for index in segment:
            before[index], after[index] = after[index], before[index]

        before[segment[-1]], after[before_segment] = before_segment, segment[-1]
        before[after_segment], after[segment[0]] = segment[0], after_segment

        return self.cls(after=after, before=before, cost=cost)

    def find_best_candidate(self, *, pool: pool.Pool) -> Optional[PathSolution]:
        concurrency = os.cpu_count() or 1
        solution = self._solution

        args: List[IPCBundle[SegmentReverse, List[List[int]]]] = [IPCBundle(self, []) for _ in range(concurrency)]
        args_index_iteration = itertools.cycle(range(concurrency))

        path = solution.get_path()
        for start in range(solution.dimension):
            segment = []
            for d in range(self._segment_length):
                segment.append(path[(start + d) % solution.dimension])

            args[next(args_index_iteration)].data.append(segment)

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
    def static_find_best_candidate(bundle: IPCBundle[SegmentReverse, List[List[int]]]) -> Tuple[Optional[PathSolution], Optional[Tuple[int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood._ensure_imported_data()

        result: Optional[PathSolution] = None
        min_pair: Optional[Tuple[int, int]] = None
        for segment in bundle.data:
            shifted = neighborhood.reverse(segment)
            if result is None or shifted < result:
                result = shifted
                min_pair = (segment[0], segment[-1])

        return result, min_pair
