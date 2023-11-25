from unittest.mock import MagicMock, patch

import pytest

from ts import tsp


def test_unknown_problem() -> None:
    with pytest.raises(tsp.ProblemNotFound):
        tsp.TSPPathSolution.import_problem("hanoi69")


def test_unsupported_edge_weight_type() -> None:
    with pytest.raises(tsp.ProblemParsingException) as exception:
        tsp.TSPPathSolution.import_problem("att48")

    assert isinstance(exception.value.original, tsp.UnsupportedEdgeWeightType)


def test_unknown_optimal_solution() -> None:
    with pytest.raises(tsp.OptimalSolutionNotFound):
        tsp.TSPPathSolution.import_problem("0000")
        tsp.TSPPathSolution.read_optimal_solution()


@patch("matplotlib.pyplot.show")
def test_solution_plot(mock: MagicMock) -> None:
    tsp.TSPPathSolution.import_problem("a280")
    solution = tsp.TSPPathSolution.initial()
    solution.plot()
    mock.assert_called_once()
