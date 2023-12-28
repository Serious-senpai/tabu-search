from unittest.mock import MagicMock, patch

import pytest

from ts import d2d


def test_unknown_problem() -> None:
    with pytest.raises(d2d.ProblemImportException):
        d2d.D2DPathSolution.import_problem(
            "69.69.69",
            drone_config=0,
            energy_mode="linear",
        )


@patch("matplotlib.pyplot.show")
def test_solution_plot(mock: MagicMock) -> None:
    d2d.D2DPathSolution.import_problem(
        "20.5.3",
        drone_config=0,
        energy_mode="non-linear",
    )
    solution = d2d.D2DPathSolution.initial()
    solution.plot()
    mock.assert_called_once()
