from __future__ import annotations

import random
import threading
from functools import partial
from multiprocessing import Pool, pool as p
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple, Union, TYPE_CHECKING

from matplotlib import axes, pyplot
from tqdm import tqdm
if TYPE_CHECKING:
    from typing_extensions import Self

from .costs import BaseMulticostComparison, ParetoSet
from ..bases import BaseSolution
from ...utils import ngettext
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
    def before_iteration(cls, iteration: int, last_improved: int, current: List[Self], pareto_costs: Dict[Tuple[float, ...], int]) -> None:
        """A hook to be called before each iteration

        The default implementation does nothing

        Parameters
        -----
        iteration:
            The current tabu search iteration, starting from 0
        last_improved:
            The last iteration when Pareto front update occured
        current:
            The current solutions
        pareto_costs:
            The counter of costs of current Pareto-optimal solutions
        """
        return

    @classmethod
    def after_iteration(cls, iteration: int, last_improved: int, current: List[Self], pareto_costs: Dict[Tuple[float, ...], int]) -> None:
        """A hook to be called after each iteration

        The default implementation does nothing

        Parameters
        -----
        iteration:
            The current tabu search iteration, starting from 0
        last_improved:
            The last iteration when Pareto front update occured
        current:
            The current solutions
        pareto_costs:
            The counter of costs of current Pareto-optimal solutions
        """
        return

    @classmethod
    def tabu_search(
        cls,
        *,
        pool_size: int,
        iterations_count: int,
        use_tqdm: bool,
        propagation_priority_key: Optional[Callable[[Dict[Tuple[float, ...], int], Tuple[float, ...], Tuple[float, ...], Self], float]] = None,
        max_propagation: Union[int, Callable[[int, Dict[Tuple[float, ...], int]], int]],
        plot_pareto_front: bool = False,
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
            A function taking 4 arguments:
            - The counter of costs of current Pareto-optimal solutions
            - The minimum cost of each dimension among the iterated solutions
            - The maximum cost of each dimension among the iterated solutions
            - The solution S
            The less the returned value, the more likely the solution S will be added to the propagation tree.
        max_propagation:
            An integer of a function taking 2 arguments:
            - The index of the current iteration (starting from 0)
            - The counter of costs of current Pareto-optimal solutions
            Denotes the maximum number of propagating solutions in an iteration
        plot_pareto_front:
            Plot the Pareto front for 2-objective optimization problems only, default to False

        Returns
        -----
        The Pareto front among the iterated solutions

        See also
        -----
        - https://en.wikipedia.org/wiki/Pareto_efficiency
        - https://en.wikipedia.org/wiki/Pareto_front
        """
        initial = cls.initial()
        results = ParetoSet([initial])
        iterations: Union[range, tqdm[int]] = range(iterations_count)
        if use_tqdm:
            iterations = tqdm(iterations, ascii=" █")

        current = [initial]
        candidate_costs = {initial.cost()} if plot_pareto_front else None
        dimensions = len(initial.cost())
        if dimensions != 2:
            message = "Cannot plot the Pareto front when the number of objectives is not 2"
            raise ValueError(message)

        extremes = [initial.cost()] * 2
        lock = threading.Lock()
        last_improved = 0
        with Pool(pool_size) as pool:
            for iteration in iterations:
                cls.before_iteration(iteration, last_improved, current, results.counter())

                if isinstance(iterations, tqdm):
                    solution_display = ngettext(len(results) == 1, "solution", "solutions")
                    iterations.set_description_str(f"Tabu search ({len(current)}/{len(results)} {solution_display})")

                propagate: List[Self] = []

                def process_solution(solution: Self) -> bool:
                    neighborhoods = list(solution.get_neighborhoods())
                    random.shuffle(neighborhoods)

                    improved = False
                    for neighborhood in neighborhoods:
                        propagated = False

                        for candidate in neighborhood.find_best_candidates(pool=pool, pool_size=pool_size):
                            with lock:
                                if candidate_costs is not None:
                                    candidate_costs.add(candidate.cost())

                                if candidate.add_to_pareto_set(results)[0]:
                                    improved = True

                                if candidate.to_propagate:
                                    propagated = True
                                    propagate.append(candidate)

                                extremes[0] = tuple(min(m, c) for m, c in zip(extremes[0], candidate.cost()))
                                extremes[1] = tuple(max(m, c) for m, c in zip(extremes[1], candidate.cost()))

                        if propagated:
                            break

                    return improved

                with p.ThreadPool(min(pool_size, len(current))) as thread_pool:
                    if any(thread_pool.map(process_solution, current)):
                        last_improved = iteration

                    thread_pool.close()
                    thread_pool.join()

                propagate = list(set(propagate))
                if len(propagate) == 0:
                    propagate = [solution.shuffle(use_tqdm=use_tqdm) for solution in current]

                pareto_counter = results.counter()
                if propagation_priority_key is not None:
                    propagate.sort(key=partial(propagation_priority_key, pareto_counter, *extremes))

                else:
                    random.shuffle(propagate)

                if isinstance(max_propagation, int):
                    max_propagation_value = max_propagation
                else:
                    max_propagation_value = max_propagation(iteration, pareto_counter)

                current = propagate[:max_propagation_value]

                cls.after_iteration(iteration, last_improved, current, results.counter())

            post_optimized_results = set(r.post_optimization(pool=pool, pool_size=pool_size, use_tqdm=use_tqdm) for r in results)

            pool.close()
            pool.join()

        if len(set(results)) != len(results):
            message = f"Check {results.__class__.__name__}.__len__ implementation"
            raise RuntimeError(message)

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

            pyplot.xlabel("Objective 1")
            pyplot.ylabel("Objective 2")
            pyplot.legend()
            pyplot.show()
            pyplot.close()

        return post_optimized_results
