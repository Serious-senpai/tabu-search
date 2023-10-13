from __future__ import annotations

import itertools
from collections import deque
from multiprocessing import pool
from typing import ClassVar, Deque, List, Optional, Tuple, Set, TYPE_CHECKING

from .base import BaseTSPNeighborhood
from ...bundle import IPCBundle
if TYPE_CHECKING:
    from ..solutions import TSPPathSolution


__all__ = ("SegmentShift",)


class SegmentShift(BaseTSPNeighborhood[Tuple[int, int, int]]):

    __slots__ = (
        "_segment_length",
    )
    _maxlen: ClassVar[int] = 100
    _tabu_list: ClassVar[Deque[Tuple[int, int, int]]] = deque()
    _tabu_set: ClassVar[Set[Tuple[int, int, int]]] = set()
    if TYPE_CHECKING:
        _segment_length: int

    def __init__(self, solution: TSPPathSolution, *, segment_length: int) -> None:
        super().__init__(solution)
        self._segment_length = segment_length
        if segment_length > solution.dimension + 2:
            raise ValueError(f"Segment length {segment_length} is too low.")

    def insert_after(self, segment_first: int, segment_last: int, x: int) -> TSPPathSolution:
        solution = self._solution

        before = list(solution.before)
        after = list(solution.after)

        before_segment = before[segment_first]
        after_segment = after[segment_last]
        after_x = after[x]

        cost = (
            solution.cost()
            + solution.distances[before_segment][after_segment]
            + solution.distances[x][segment_first] + solution.distances[segment_last][after_x]
            - solution.distances[before_segment][segment_first] - solution.distances[segment_last][after_segment]
            - solution.distances[x][after_x]
        )

        after[before_segment], before[after_segment] = after_segment, before_segment
        after[x], before[segment_first] = segment_first, x
        after[segment_last], before[after_x] = after_x, segment_last

        return self.cls(after=after, before=before, cost=cost)

    def find_best_candidate(self, *, pool: pool.Pool, pool_size: int) -> Optional[TSPPathSolution]:
        solution = self._solution

        args: List[IPCBundle[SegmentShift, List[Tuple[int, int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
        args_index_iteration = itertools.cycle(range(pool_size))

        for segment_first_index in range(solution.dimension):
            segment_end_index = (segment_first_index + self._segment_length - 1) % solution.dimension
            for d in range(solution.dimension - self._segment_length - 1):
                index = (segment_end_index + d + 1) % solution.dimension
                args[next(args_index_iteration)].data.append((solution.path[segment_first_index], solution.path[segment_end_index], solution.path[index]))

        result: Optional[TSPPathSolution] = None
        min_pair: Optional[Tuple[int, int, int]] = None
        for result_temp, min_pair_temp in pool.imap(self.static_find_best_candidate, args):
            if result_temp is None or min_pair_temp is None:
                continue

            if result is None or result_temp < result:
                result = result_temp
                min_pair = min_pair_temp

        if min_pair is not None:
            self.add_to_tabu(min_pair)

        return result

    @staticmethod
    def static_find_best_candidate(bundle: IPCBundle[SegmentShift, List[Tuple[int, int, int]]]) -> Tuple[Optional[TSPPathSolution], Optional[Tuple[int, int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood._ensure_imported_data()

        result: Optional[TSPPathSolution] = None
        min_args: Optional[Tuple[int, int, int]] = None
        for args in bundle.data:
            shifted = neighborhood.insert_after(*args)
            if result is None or shifted < result:
                result = shifted
                min_args = args

        return result, min_args
