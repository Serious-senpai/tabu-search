from __future__ import annotations

import platform
import os
import threading
import sys
from typing import Any, Callable, Literal, ParamSpec, TypeVar


__all__ = (
    "false",
    "true",
    "zero",
    "ngettext",
    "display_platform",
    "synchronized",
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


_P = ParamSpec("_P")
_T = TypeVar("_T")


def synchronized(func: Callable[_P, _T], /) -> Callable[_P, _T]:
    lock = threading.Lock()

    def wrapper(*args: _P.args, **kwargs: _P.kwargs) -> _T:
        with lock:
            return func(*args, **kwargs)

    return wrapper
