from __future__ import annotations

import threading
from collections import deque
from multiprocessing import pool
from typing import (
    Any,
    ClassVar,
    Deque,
    Dict,
    Final,
    Generic,
    Sequence,
    Set,
    Type,
    TypeVar,
    TYPE_CHECKING,
    final,
)

if TYPE_CHECKING:
    from typing_extensions import Self


__all__ = ()


class BaseSolution:

    __slots__ = ()

    def get_neighborhoods(self) -> Sequence[BaseNeighborhood[Self, Any]]:
        """Returns all neighborhoods of the current solution

        Subclasses must implement this.
        """
        raise NotImplementedError

    def shuffle(self, *, use_tqdm: bool) -> Self:
        """Shuffle the current solution

        The default implementation does nothing.

        Parameters
        -----
        use_tqdm:
            Whether to display the progress bar
        """
        return self

    def post_optimization(
        self,
        *,
        pool: pool.Pool,
        pool_size: int,
        use_tqdm: bool,
    ) -> Self:
        """Perform post-optimization for this solution

        The default implementation does nothing.

        Parameters
        -----
        pool:
            The process pool to perform post-optimization
        pool_size:
            The process pool size
        use_tqdm:
            Whether to display the progress bar
        """
        return self

    @classmethod
    def initial(cls) -> Self:
        """Generate the initial solution for tabu search

        Subclasses must implement this.
        """
        raise NotImplementedError

    def __hash__(self) -> int:
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} hash={self.__hash__()}>"


_ST = TypeVar("_ST", bound=BaseSolution, covariant=True)
_TT = TypeVar("_TT")


class BaseNeighborhood(Generic[_ST, _TT]):

    __slots__ = (
        "_solution",
        "cls",
        "extras",
    )
    if TYPE_CHECKING:
        # https://github.com/python/mypy/issues/8982
        # https://stackoverflow.com/a/75160662

        _maxlen: ClassVar[int]
        _tabu_list: ClassVar[Deque[_TT]]  # type: ignore
        _tabu_lock: ClassVar[threading.Lock] = threading.Lock()
        tabu_set: ClassVar[Set[_TT]]  # type: ignore

    def __init__(self, solution: _ST, /) -> None:
        self._solution: Final[_ST] = solution
        self.cls: Final[Type[_ST]] = type(solution)
        self.extras: Final[Dict[Any, Any]] = {}

    @final
    def __init_subclass__(cls, *args: Any, **kwargs: Any) -> None:
        super().__init_subclass__(*args, **kwargs)
        cls._maxlen = 10
        cls._tabu_list = deque()
        cls._tabu_lock = threading.Lock()
        cls.tabu_set = set()

    @final
    @classmethod
    def add_to_tabu(cls, target: _TT) -> None:
        with cls._tabu_lock:
            if target in cls.tabu_set:
                rotated = 0
                while cls._tabu_list[0] != target:
                    cls._tabu_list.rotate(-1)
                    rotated += 1

                cls._tabu_list.popleft()
                cls._tabu_list.rotate(rotated)

                cls._tabu_list.append(target)

            else:
                cls.tabu_set.add(target)
                cls._tabu_list.append(target)
                cls.__remove_from_tabu()

    @final
    @classmethod
    def __remove_from_tabu(cls) -> None:
        while len(cls.tabu_set) > cls._maxlen:
            cls.tabu_set.remove(cls._tabu_list.popleft())

    @final
    @classmethod
    def reset_tabu(cls, *, maxlen: int = 10) -> None:
        with cls._tabu_lock:
            cls._maxlen = maxlen
            cls.__remove_from_tabu()
