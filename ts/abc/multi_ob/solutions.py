from __future__ import annotations

import random
import time
import threading
from functools import partial
from multiprocessing import Pool, pool as p
from typing import Any, Callable, List, Optional, Sequence, Set, Tuple, Union, TYPE_CHECKING

from matplotlib import axes, pyplot
from tqdm import tqdm
if TYPE_CHECKING:
    from typing_extensions import Self

from .costs import BaseMulticostComparison
from ..bases import BaseSolution
from ...utils import ngettext, synchronized
if TYPE_CHECKING:
    from .neighborhoods import MultiObjectiveNeighborhood


__all__ = ("MultiObjectiveSolution",)


class MultiObjectiveSolution(BaseSolution, BaseMulticostComparison):
    """Base class for solutions to a multi-objective optimization problem"""

    __slots__ = ()

    @property
    def to_propagate(self) -> bool:
        """Whether the tabu search should use this solution for propagation in the next iteration.

        Subclasses must implement this.
        """
        raise NotImplementedError

    @to_propagate.setter
    def to_propagate(self, propagate: bool) -> None:
        raise NotImplementedError

    def get_neighborhoods(self) -> Sequence[MultiObjectiveNeighborhood[Self, Any]]:
        raise NotImplementedError

    @classmethod
    def tabu_search(
        cls,
        *,
        pool_size: int,
        iterations_count: int,
        use_tqdm: bool,
        propagation_priority_key: Optional[Callable[[Set[Tuple[float, ...]], Self], float]] = None,
        max_propagation: int,
        plot_pareto_front: bool = False,
        logger: Optional[Callable[[str], Any]] = None,
    ) -> Set[Self]:
        """Run the tabu search algorithm to find the Pareto front for this multi-objective optimization problem.

        Parameters
        -----
        pool_size:
            The size of the process pool to perform parallelism
        iterations_count:
            The number of iterations to improve from the initial solution
        use_tqdm:
            Whether to display the progress bar
        propagation_priority_key:
            A function taking 2 arguments: The first one is the set of costs of current Pareto-optimal solutions, the second one is
            the solution S. The less the returned value, the more likely the solution S will be added to the propagation tree.
        max_propagation:
            An integer or a function that takes the current Pareto front as a single parameter and return the maximum number of
            propagating solutions at a time
        plot_pareto_front:
            Plot the Pareto front for 2-objective optimization problems only, default to False
        logger:
            The logging function taking a single str argument

        Returns
        -----
        The Pareto front among the iterated solutions

        See also
        -----
        - https://en.wikipedia.org/wiki/Pareto_efficiency
        - https://en.wikipedia.org/wiki/Pareto_front
        """
        initial = cls.initial()
        results = {initial}
        iterations: Union[range, tqdm[int]] = range(iterations_count)
        if use_tqdm:
            iterations = tqdm(iterations, ascii=" █")

        if logger is not None:
            logger = synchronized(logger)

        current = [initial]
        candidate_costs = {initial.cost()} if plot_pareto_front else None
        if len(initial.cost()) != 2:
            message = f"Cannot plot the Pareto front when the number of objectives is not 2"
            raise ValueError(message)

        with Pool(pool_size) as pool:
            lock = threading.Lock()
            last_improved = 0
            start = time.perf_counter()
            for iteration in iterations:
                if isinstance(iterations, tqdm):
                    solution_display = ngettext(len(results) == 1, "solution", "solutions")
                    iterations.set_description_str(f"Tabu search ({len(current)}/{len(results)} {solution_display})")

                if logger is not None:
                    logger(f"Iteration #{iteration + 1}/{iterations_count},Solutions count,{len(results)},Timer (s),{time.perf_counter() - start:.4f}\n")

                propagate: List[Self] = []

                def process_solution(solution: Self) -> bool:
                    neighborhoods = list(solution.get_neighborhoods())
                    random.shuffle(neighborhoods)

                    improved = False
                    for neighborhood in neighborhoods:
                        propagated = False

                        for candidate in neighborhood.find_best_candidates(pool=pool, pool_size=pool_size, logger=logger):
                            with lock:
                                if candidate_costs is not None:
                                    candidate_costs.add(candidate.cost())

                                improved = improved or candidate.add_to_pareto_set(results)

                                if candidate.to_propagate:
                                    propagated = True
                                    propagate.append(candidate)

                        if propagated:
                            break

                    return improved

                with p.ThreadPool(min(pool_size, len(current))) as thread_pool:
                    if any(thread_pool.map(process_solution, current)):
                        last_improved = iteration

                    thread_pool.close()
                    thread_pool.join()

                if len(propagate) == 0:
                    propagate = [solution.shuffle(use_tqdm=use_tqdm, logger=logger) for solution in current]

                if propagation_priority_key is not None:
                    propagate.sort(key=partial(propagation_priority_key, set(s.cost() for s in results)))

                current = propagate[:max_propagation]

                if logger is not None:
                    logger(f"Last improvement: #{last_improved + 1}/{iterations_count}\n")

            results = set(r.post_optimization(pool=pool, pool_size=pool_size, use_tqdm=use_tqdm, logger=logger) for r in results)

            pool.close()
            pool.join()

        if candidate_costs is not None:
            _, ax = pyplot.subplots()
            assert isinstance(ax, axes.Axes)

            ax.scatter(
                [cost[0] for cost in candidate_costs],
                [cost[1] for cost in candidate_costs],
                c="gray",
                label=f"Distinct costs found ({len(candidate_costs)})",
            )

            result_costs = set(result.cost() for result in results)
            ax.scatter(
                [cost[0] for cost in result_costs],
                [cost[1] for cost in result_costs],
                c="red",
                label=f"Pareto front, distinct costs ({len(result_costs)})",
            )

            ax.grid(True)

            pyplot.legend()
            pyplot.show()

        return results
