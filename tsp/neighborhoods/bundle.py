from __future__ import annotations

from typing import Generic, TypeVar, TYPE_CHECKING

from .base import BasePathNeighborhood


_T = TypeVar("_T")
_NT = TypeVar("_NT", bound=BasePathNeighborhood)


class IPCBundle(Generic[_NT, _T]):

    __slots__ = (
        "neighborhood",
        "data",
    )
    if TYPE_CHECKING:
        neighborhood: _NT
        data: _T

    def __init__(self, neighborhood: _NT, data: _T) -> None:
        self.neighborhood = neighborhood
        self.data = data
