from __future__ import annotations

from collections import deque
from multiprocessing import pool
from typing import (
    Any,
    ClassVar,
    Deque,
    Dict,
    Generic,
    Protocol,
    Set,
    Tuple,
    Type,
    TypeVar,
    TYPE_CHECKING,
    final,
    runtime_checkable,
)

if TYPE_CHECKING:
    from typing_extensions import Self


__all__ = ("SupportsRichComparison",)


@runtime_checkable
class SupportsRichComparison(Protocol):
    def __eq__(self, other: Any) -> bool: ...
    def __ne__(self, other: Any) -> bool: ...
    def __lt__(self, other: Any) -> bool: ...
    def __le__(self, other: Any) -> bool: ...
    def __gt__(self, other: Any) -> bool: ...
    def __ge__(self, other: Any) -> bool: ...


class _BaseSolution:

    __slots__ = ()

    def get_neighborhoods(self) -> Tuple[_BaseNeighborhood[Self, Any], ...]:
        """Returns all neighborhoods of the current solution"""
        raise NotImplementedError

    def shuffle(self, *, use_tqdm: bool) -> Self:
        """Shuffle the current solution

        The default implementation does nothing.

        Parameters
        -----
        use_tqdm: `bool`
            Whether to display the progress bar
        """
        return self

    def post_optimization(self, *, pool: pool.Pool, pool_size: int, use_tqdm: bool) -> Self:
        """Perform post-optimization for this solution

        The default implementation does nothing.

        Parameters
        -----
        pool: `pool.Pool`
            The process pool to perform post-optimization
        pool_size: `int`
            The process pool size
        use_tqdm: `bool`
            Whether to display the progress bar
        """
        return self

    @classmethod
    def initial(cls) -> Self:
        """Generate the initial solution for tabu search"""
        raise NotImplementedError

    def __hash__(self) -> int:
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} hash={self.__hash__()}>"


_ST = TypeVar("_ST", bound=_BaseSolution)
_TT = TypeVar("_TT")


class _BaseNeighborhood(Generic[_ST, _TT]):

    __slots__ = (
        "_solution",
        "cls",
        "extras",
    )
    if TYPE_CHECKING:
        _solution: _ST
        cls: Type[_ST]
        extras: Dict[Any, Any]

        _maxlen: ClassVar[int]
        _tabu_list: ClassVar[Deque[_TT]]  # type: ignore
        _tabu_set: ClassVar[Set[_TT]]  # type: ignore

    def __init__(self, solution: _ST, /) -> None:
        self._solution = solution
        self.cls = type(solution)
        self.extras = {}

    @final
    def __init_subclass__(cls, *args: Any, **kwargs: Any) -> None:
        super().__init_subclass__(*args, **kwargs)
        cls._maxlen = 10
        cls._tabu_list = deque(maxlen=cls._maxlen)
        cls._tabu_set = set()

    @final
    @classmethod
    def is_tabu(cls, __o: _TT, /) -> bool:
        return __o in cls._tabu_set

    @final
    @classmethod
    def add_to_tabu(cls, target: _TT) -> None:
        cls._tabu_set.add(target)
        cls._tabu_list.append(target)
        cls.remove_from_tabu()

    @final
    @classmethod
    def remove_from_tabu(cls) -> None:
        while len(cls._tabu_set) > cls._maxlen:
            try:
                cls._tabu_set.remove(cls._tabu_list.popleft())
            except KeyError:
                pass

    @final
    @classmethod
    def reset_tabu(cls, *, maxlen: int = 10) -> None:
        cls._maxlen = maxlen
        cls.remove_from_tabu()
