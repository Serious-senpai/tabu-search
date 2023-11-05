from typing import Any


class TabuSearchException(Exception):
    """Base class for all exceptions from this library"""
    pass


class AlreadyInTabu(TabuSearchException):
    """Exception raised when an operation is already in the tabu list"""

    def __init__(self, target: Any, /) -> None:
        super().__init__(f"Already in tabu list: {target!r}")
