from __future__ import annotations

import random
from multiprocessing import Pool
from typing import Any, Sequence, Union, TYPE_CHECKING

from tqdm import tqdm
if TYPE_CHECKING:
    from typing_extensions import Self

from .costs import BaseCostComparison
from ..types import BaseSolution
if TYPE_CHECKING:
    from .neighborhoods import SingleObjectiveNeighborhood


__all__ = ("SingleObjectiveSolution",)


class SingleObjectiveSolution(BaseSolution, BaseCostComparison):
    """Base class for solutions to a single-objective optimization problem"""

    __slots__ = ()

    def get_neighborhoods(self) -> Sequence[SingleObjectiveNeighborhood[Self, Any]]:
        raise NotImplementedError

    @classmethod
    def tabu_search(cls, *, pool_size: int, iterations_count: int, use_tqdm: bool, shuffle_after: int) -> Self:
        """Run the tabu search algorithm to find the best solution to this single-objective optimization problem.

        Parameters
        -----
        pool_size: `int`
            The size of the process pool to perform parallelism
        iterations_count: `int`
            The number of iterations to improve from the initial solution
        use_tqdm: `bool`
            Whether to display the progress bar
        shuffle_after: `int`
            After the specified number of non-improving iterations, shuffle the current solution

        Returns
        -----
        `SingleObjectiveSolution`
            The solution with the lowest cost among the iterated ones.
        """
        result = current = cls.initial()
        iterations: Union[range, tqdm[int]] = range(iterations_count)
        if use_tqdm:
            iterations = tqdm(iterations, ascii=" █")

        with Pool(pool_size) as pool:
            last_improved = 0
            for iteration in iterations:
                if isinstance(iterations, tqdm):
                    iterations.set_description_str(f"Tabu search ({current.cost()}/{result.cost()})")

                neighborhoods = current.get_neighborhoods()
                best_candidate = random.choice(neighborhoods).find_best_candidate(pool=pool, pool_size=pool_size)
                if best_candidate is None:
                    break

                if best_candidate < current:
                    last_improved = iteration

                current = best_candidate
                result = min(result, current)

                if iteration - last_improved >= shuffle_after:
                    current = current.shuffle(use_tqdm=use_tqdm)
                    last_improved = iteration

            return result.post_optimization(pool=pool, pool_size=pool_size, use_tqdm=use_tqdm)
