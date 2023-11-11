from __future__ import annotations

import platform
import math
import os
import threading
import sys
from typing import Any, Callable, Literal, ParamSpec, Sequence, TypeVar, TYPE_CHECKING, overload


__all__ = (
    "false",
    "true",
    "zero",
    "ngettext",
    "display_platform",
    "synchronized",
    "isclose",
)


def false(*args: Any, **kwargs: Any) -> Literal[False]:
    return False


def true(*args: Any, **kwargs: Any) -> Literal[True]:
    return True


def zero(*args: Any, **kwargs: Any) -> Literal[0]:
    return 0


def ngettext(predicate: bool, if_true: str, if_false: str, /) -> str:
    return if_true if predicate else if_false


def display_platform() -> None:
    cpu_count = os.cpu_count() or 1

    display = f"Running on {sys.platform} with {cpu_count} " + ngettext(cpu_count == 1, "CPU", "CPUs") + "\n"
    display += f"Python {sys.version}\n"
    display += ", ".join((platform.platform(), platform.processor())) + "\n"
    display += "-" * 30

    print(display)


if TYPE_CHECKING:
    _P = ParamSpec("_P")
    _T = TypeVar("_T")


def synchronized(func: Callable[_P, _T], /) -> Callable[_P, _T]:
    lock = threading.Lock()

    def wrapper(*args: _P.args, **kwargs: _P.kwargs) -> _T:
        with lock:
            return func(*args, **kwargs)

    return wrapper


@overload
def isclose(
    first: float,
    second: float,
    /,
) -> bool: ...


@overload
def isclose(
    first: Sequence[float],
    second: Sequence[float],
    /,
) -> bool: ...


@overload
def isclose(
    first: Sequence[Sequence[float]],
    second: Sequence[Sequence[float]],
    /,
) -> bool: ...


@overload
def isclose(
    first: Sequence[Sequence[Sequence[float]]],
    second: Sequence[Sequence[Sequence[float]]],
    /,
) -> bool: ...


@overload
def isclose(
    first: Sequence[Sequence[Sequence[Sequence[float]]]],
    second: Sequence[Sequence[Sequence[Sequence[float]]]],
    /,
) -> bool: ...


@overload
def isclose(
    first: Sequence[Sequence[Sequence[Sequence[Sequence[float]]]]],
    second: Sequence[Sequence[Sequence[Sequence[Sequence[float]]]]],
    /,
) -> bool: ...


def isclose(first: Any, second: Any, /) -> bool:
    try:
        return all(isclose(f, s) for f, s in zip(first, second))
    except TypeError:
        return math.isclose(first, second, rel_tol=0.001, abs_tol=0.001)
