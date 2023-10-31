from multiprocessing import pool as p
from typing import Iterable, Set, TYPE_CHECKING

from .mixins import D2DNeighborhoodMixin
from ...abc import MultiObjectiveNeighborhood
if TYPE_CHECKING:
    from ..solutions import D2DPathSolution


if TYPE_CHECKING:
    _BaseNeighborhood = MultiObjectiveNeighborhood[D2DPathSolution, ]
else:
    _BaseNeighborhood = MultiObjectiveNeighborhood



class Swappoint(D2DNeighborhoodMixin, _BaseNeighborhood):

    __slots__ = (
        "length",
    )

    