import pytest

from ts import d2d


def test_unknown_problem() -> None:
    with pytest.raises(d2d.ProblemImportException):
        d2d.D2DPathSolution.import_problem(
            "69.69.69",
            drone_config_mapping=(0, 0, 0, 0),
            energy_mode=d2d.DroneEnergyConsumptionMode.LINEAR,
        )
