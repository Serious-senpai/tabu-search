import platform
import os
import sys
from typing import Any, Literal


__all__ = (
    "false",
    "true",
    "zero",
    "display_platform",
)


def false(*args: Any, **kwargs: Any) -> Literal[False]:
    return False


def true(*args: Any, **kwargs: Any) -> Literal[True]:
    return True


def zero(*args: Any, **kwargs: Any) -> Literal[0]:
    return 0


def display_platform() -> None:
    print(
        f"Running on {sys.platform}\nPython {sys.version}\nCPU count = {os.cpu_count()}\n"
        + ", ".join((platform.platform(), platform.processor()))
    )
