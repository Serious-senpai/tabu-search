from __future__ import annotations

import itertools
from multiprocessing import pool
from typing import List, Optional, Tuple, TYPE_CHECKING

from .base import TSPBaseNeighborhood
from ...bundle import IPCBundle
if TYPE_CHECKING:
    from ..solutions import TSPPathSolution


__all__ = ("Swap",)


class Swap(TSPBaseNeighborhood[Tuple[int, int, int, int]]):

    __slots__ = (
        "_first_length",
        "_second_length",
    )
    if TYPE_CHECKING:
        _first_length: int
        _second_length: int

    def __init__(self, solution: TSPPathSolution, *, first_length: int, second_length: int) -> None:
        super().__init__(solution)
        self._first_length = first_length
        self._second_length = second_length

    def swap(self, first_head: int, first_tail: int, second_head: int, second_tail: int) -> TSPPathSolution:
        solution = self._solution

        before = list(solution.before)
        after = list(solution.after)

        if first_head == after[second_tail]:
            first_head, first_tail, second_head, second_tail = second_head, second_tail, first_head, first_tail

        if first_tail == before[second_head]:
            before_first = before[first_head]
            after_second = after[second_tail]

            cost = (
                solution.cost()
                + solution.distances[before_first][second_head]
                + solution.distances[second_tail][first_head]
                + solution.distances[first_tail][after_second]
                - solution.distances[before_first][first_head]
                - solution.distances[first_tail][second_head]
                - solution.distances[second_tail][after_second]
            )

            after[before_first], before[second_head] = second_head, before_first
            after[second_tail], before[first_head] = first_head, second_tail
            after[first_tail], before[after_second] = after_second, first_tail

        else:
            before_first = before[first_head]
            before_second = before[second_head]
            after_first = after[first_tail]
            after_second = after[second_tail]

            cost = (
                solution.cost()
                + solution.distances[before_first][second_head] + solution.distances[second_tail][after_first]
                + solution.distances[before_second][first_head] + solution.distances[first_tail][after_second]
                - solution.distances[before_first][first_head] - solution.distances[first_tail][after_first]
                - solution.distances[before_second][second_head] - solution.distances[second_tail][after_second]
            )

            after[before_first], before[second_head] = second_head, before_first
            after[before_second], before[first_head] = first_head, before_second
            after[second_tail], before[after_first] = after_first, second_tail
            after[first_tail], before[after_second] = after_second, first_tail

        return self.cls(after=tuple(after), before=tuple(before), cost=cost)

    def find_best_candidate(self, *, pool: pool.Pool, pool_size: int) -> Optional[TSPPathSolution]:
        solution = self._solution

        bundles: List[IPCBundle[Swap, List[Tuple[int, int, int, int]]]] = [IPCBundle(self, []) for _ in range(pool_size)]
        bundle_iter = itertools.cycle(bundles)

        for first_head_index in range(solution.dimension):
            first_tail_index = (first_head_index + self._first_length - 1) % solution.dimension

            for d in range(solution.dimension - self._first_length - self._second_length + 1):
                second_head_index = (first_tail_index + d + 1) % solution.dimension
                second_tail_index = (second_head_index + self._second_length - 1) % solution.dimension

                # Guaranteed order: first_head - first_tail - second_head - second_tail
                arg = (
                    solution.path[first_head_index],
                    solution.path[first_tail_index],
                    solution.path[second_head_index],
                    solution.path[second_tail_index],
                )
                next(bundle_iter).data.append(arg)

        result: Optional[TSPPathSolution] = None
        min_swap: Optional[Tuple[int, int, int, int]] = None
        for result_temp, min_swap_temp in pool.imap_unordered(self.static_find_best_candidate, bundles):
            if result_temp is None or min_swap_temp is None:
                continue

            if min_swap_temp in self.tabu_set:
                continue

            if result is None or result_temp < result:
                result = result_temp
                min_swap = min_swap_temp

        if min_swap is not None:
            self.add_to_tabu(min_swap)

        return result

    @staticmethod
    def static_find_best_candidate(bundle: IPCBundle[Swap, List[Tuple[int, int, int, int]]]) -> Tuple[Optional[TSPPathSolution], Optional[Tuple[int, int, int, int]]]:
        neighborhood = bundle.neighborhood
        neighborhood.ensure_imported_data()

        result: Optional[TSPPathSolution] = None
        min_swap: Optional[Tuple[int, int, int, int]] = None
        for swap in bundle.data:
            swapped = neighborhood.swap(*swap)
            if result is None or swapped < result:
                result = swapped
                min_swap = swap

        return result, min_swap
