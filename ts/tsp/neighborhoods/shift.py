from __future__ import annotations

import itertools
from multiprocessing import pool
from typing import List, Optional, Tuple, TYPE_CHECKING

from .base import TSPBaseNeighborhood
from ...bundle import IPCBundle
if TYPE_CHECKING:
    from ..solutions import TSPPathSolution


__all__ = ("SegmentShift",)


class SegmentShift(TSPBaseNeighborhood[Tuple[int, int, int]]):

    __slots__ = (
        "_segment_length",
    )
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

        return self.cls(after=tuple(after), before=tuple(before), cost=cost)

    def find_best_candidate(self, *, pool: pool.Pool, pool_size: int) -> Optional[TSPPathSolution]:
        solution = self._solution

        bundles: List[IPCBundle[SegmentShift, List[Tuple[int, int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
        bundle_iter = itertools.cycle(bundles)

        for segment_first_index in range(solution.dimension):
            segment_end_index = (segment_first_index + self._segment_length - 1) % solution.dimension
            for d in range(solution.dimension - self._segment_length - 1):
                index = (segment_end_index + d + 1) % solution.dimension
                next(bundle_iter).data.append((solution.path[segment_first_index], solution.path[segment_end_index], solution.path[index]))

        result: Optional[TSPPathSolution] = None
        min_pair: Optional[Tuple[int, int, int]] = None
        for result_temp, min_pair_temp in pool.imap_unordered(self.static_find_best_candidate, bundles):
            if result_temp is None or min_pair_temp is None:
                continue

            if min_pair_temp in self.tabu_set:
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
        neighborhood.ensure_imported_data()

        result: Optional[TSPPathSolution] = None
        min_args: Optional[Tuple[int, int, int]] = None
        for args in bundle.data:
            shifted = neighborhood.insert_after(*args)
            if result is None or shifted < result:
                result = shifted
                min_args = args

        return result, min_args
