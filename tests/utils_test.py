from math import sqrt

from ts import utils


def test_hv_1() -> None:
    hv = utils.hypervolume(
        [
            (100, 100),
            (100, 100),
        ],
        ref_normalized_point=(1, 1),
    )
    assert hv is not None
    assert utils.isclose(hv, 0.0)


def test_hv_2() -> None:
    hv = utils.hypervolume(
        [
            (100, 200),
            (200, 100),
        ],
        ref_normalized_point=(1, 1),
    )
    assert hv is not None
    assert utils.isclose(hv, 0.0)


def test_hv_3() -> None:
    hv = utils.hypervolume(
        [
            (100, 100),
            (100, 200),
            (200, 100),
        ],
        ref_normalized_point=(1, 1),
    )
    assert hv is not None
    assert utils.isclose(hv, 1.0)


def test_hv_4() -> None:
    hv = utils.hypervolume(
        [
            (100, 500),
            (200, 400),
            (300, 300),
            (400, 200),
            (500, 100),
        ],
        ref_normalized_point=(1, 1),
    )
    assert hv is not None
    assert utils.isclose(hv, 6 / 16)


def test_igd() -> None:
    igd = utils.inverted_generational_distance(
        [
            (100, 500),
            (200, 400),
            (300, 300),
            (400, 200),
            (500, 100),
        ],
        ref_costs=[
            (100, 200),
            (200, 100),
        ]
    )
    assert igd is not None
    assert utils.isclose(igd, sqrt(0.25 ** 2 + 0.5 ** 2))
