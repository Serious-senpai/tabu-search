from __future__ import annotations

import random
from multiprocessing import Pool
from typing import Any, Sequence, Union, TYPE_CHECKING

from tqdm import tqdm
if TYPE_CHECKING:
    from typing_extensions import Self

from .costs import BaseCostComparison
from ..bases import BaseSolution
if TYPE_CHECKING:
    from .neighborhoods import SingleObjectiveNeighborhood


__all__ = ("SingleObjectiveSolution",)


class SingleObjectiveSolution(BaseSolution, BaseCostComparison):
    """Base class for solutions to a single-objective optimization problem"""

    __slots__ = ()

    def get_neighborhoods(self) -> Sequence[SingleObjectiveNeighborhood[Self, Any]]:
        raise NotImplementedError

    @classmethod
    def tabu_search(
        cls,
        *,
        pool_size: int,
        iterations_count: int,
        use_tqdm: bool,
        shuffle_after: int,
    ) -> Self:
        """Run the tabu search algorithm to find the best solution to this single-objective optimization problem.

        Parameters
        -----
        pool_size:
            The size of the process pool to perform parallelism
        iterations_count:
            The number of iterations to improve from the initial solution
        use_tqdm:
            Whether to display the progress bar
        shuffle_after:
            After the specified number of non-improving iterations (start counting when hitting the local optimum),
            shuffle the current solution

        Returns
        -----
        The solution with the lowest cost among the iterated ones.
        """
        result = current = cls.initial()
        iterations: Union[range, tqdm[int]] = range(iterations_count)
        if use_tqdm:
            iterations = tqdm(iterations, ascii=" █")

        with Pool(pool_size) as pool:
            last_improved = 0
            local_optimal_hit = False
            for iteration in iterations:
                if isinstance(iterations, tqdm):
                    iterations.set_description_str(f"Tabu search ({current.cost()}/{result.cost()})")

                neighborhoods = current.get_neighborhoods()
                best_candidate = random.choice(neighborhoods).find_best_candidate(pool=pool, pool_size=pool_size)
                if best_candidate is None:
                    break

                if not local_optimal_hit:
                    last_improved = iteration

                if best_candidate > current:
                    local_optimal_hit = True

                current = best_candidate
                if current < result:
                    result = current
                    last_improved = iteration

                if iteration - last_improved >= shuffle_after:
                    current = current.shuffle(use_tqdm=use_tqdm)
                    local_optimal_hit = False
                    last_improved = iteration

            result = result.post_optimization(pool=pool, pool_size=pool_size, use_tqdm=use_tqdm)

            pool.close()
            pool.join()

        return result
