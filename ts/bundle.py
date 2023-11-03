from __future__ import annotations

from typing import Generic, TypeVar, TYPE_CHECKING

from .abc.bases import BaseNeighborhood


_T = TypeVar("_T")
_NT = TypeVar("_NT", bound=BaseNeighborhood)


class IPCBundle(Generic[_NT, _T]):
    """Instances holding neighborhood additional data for IPC

    There are 2 ways to pass data to another process: the first way is to use an instance from this
    class, and the second way is to utilize `neighborhood.extras`.
    """

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

    def __repr__(self) -> str:
        return f"<IPCBundle({self.neighborhood.__class__.__name__}) data={self.data!r}>"
