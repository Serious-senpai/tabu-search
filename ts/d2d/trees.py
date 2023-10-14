from __future__ import annotations

from collections import deque
from typing import Any, Callable, Deque, Iterable, Generic, List, Optional, Set, Tuple, TypeVar, Union, TYPE_CHECKING, overload

from typing_extensions import SupportsIndex


__all__ = (
    "DynamicLengthSegmentTree",
)
_T = TypeVar("_T")


class _AutoExtendList(List[Optional[_T]]):

    def __extend(self, index: Union[SupportsIndex, slice], /) -> None:
        if isinstance(index, SupportsIndex):
            length = index.__index__() + 1
        elif index.stop is not None:
            length = index.stop
        else:
            return

        size = self.__len__()
        if size < length:
            self.extend([None] * (length - size))

    @overload
    def __getitem__(self, index: SupportsIndex, /) -> Optional[_T]: ...
    @overload
    def __getitem__(self, index: slice, /) -> _AutoExtendList[Optional[_T]]: ...

    def __getitem__(self, index: Any, /) -> Any:
        self.__extend(index)
        result = super().__getitem__(index)
        if isinstance(result, list):
            result = _AutoExtendList(result)

        return result

    @overload
    def __setitem__(self, index: SupportsIndex, value: Optional[_T], /) -> None: ...
    @overload
    def __setitem__(self, index: slice, value: Iterable[Optional[_T]], /) -> None: ...

    def __setitem__(self, index: Any, value: Any, /) -> None:
        self.__extend(index)
        super().__setitem__(index, value)

    def __repr__(self) -> str:
        return f"_AutoExtendList({super().__repr__()})"


class DynamicLengthSegmentTree(Generic[_T]):
    """Segment tree supporting insertion and deletion (although further improvements are required)"""

    __slots__ = (
        "_operator",
        "_size",
        "_tree",
    )
    if TYPE_CHECKING:
        _operator: Callable[[_T, _T], _T]
        _size: int
        _tree: _AutoExtendList[Tuple[_T, int]]

    def __init__(self, array: List[_T], *, operator: Callable[[_T, _T], _T]) -> None:
        self._operator = operator
        self._size = len(array)
        self._tree = _AutoExtendList([None] * (4 * self._size))

        def build(tree_index: int, low: int, high: int) -> None:
            tree = self._tree
            if low == high:
                tree[tree_index] = (array[low], 1)

            else:
                left = 2 * tree_index + 1
                right = 2 * tree_index + 2

                mid = (low + high) // 2
                build(left, low, mid)
                build(right, mid + 1, high)

                tree[tree_index] = self._merge(tree_index)

        build(0, 0, self._size - 1)

    def operator(self, first: Optional[_T], second: Optional[_T]) -> Optional[_T]:
        if first is None:
            return second

        if second is None:
            return first

        return self._operator(first, second)

    @overload
    def _merge(self, left: int, right: int, /) -> Optional[Tuple[_T, int]]: ...
    @overload
    def _merge(self, index: int, /) -> Optional[Tuple[_T, int]]: ...

    def _merge(self, *args: int) -> Optional[Tuple[_T, int]]:
        if len(args) == 1:
            left = 2 * args[0] + 1
            right = 2 * args[0] + 2
        elif len(args) == 2:
            left, right = args
        else:
            message = f"Invalid argument: {args}"
            raise ValueError(message)

        tree = self._tree
        left_child = tree[left]
        right_child = tree[right]

        if left_child is None:
            return right_child

        if right_child is None:
            return left_child

        return (self._operator(left_child[0], right_child[0]), left_child[1] + right_child[1])

    @staticmethod
    def _in_range(index: int, range: Tuple[int, int]) -> bool:
        return range[0] <= index <= range[1]

    def _find_lca(self, first: int, second: int) -> int:
        tree = self._tree

        result: int = 0
        queue: Deque[Tuple[int, int, int]] = deque([(0, 0, self._size - 1)])
        while len(queue) > 0:
            current, low, high = queue.popleft()
            result = current
            left = 2 * current + 1
            right = 2 * current + 2

            left_child, right_child = tree[left], tree[right]
            if left_child is not None:
                left_size = left_child[1]
                left_range = (low, low + left_size - 1)

                if self._in_range(first, left_range) and self._in_range(second, left_range):
                    queue.append((left, *left_range))

            else:
                left_size = 0

            if right_child is not None:
                right_range = (low + left_size, high)
                if self._in_range(first, right_range) and self._in_range(second, right_range):
                    queue.append((right, *right_range))

        return result

    def _update(self, changed: int, *, at_position: Optional[Callable[[int, Optional[_T]], Any]] = None) -> None:
        tree = self._tree

        def _update(tree_index: int, low: int, high: int) -> None:
            if low == high:
                if at_position is not None:
                    current = tree[tree_index]
                    at_position(tree_index, None if current is None else current[0])

            else:
                left = 2 * tree_index + 1
                right = 2 * tree_index + 2

                left_child, right_child = tree[left], tree[right]
                if left_child is not None:
                    left_size = left_child[1]
                    left_range = (low, low + left_size - 1)

                    if self._in_range(changed, left_range):
                        _update(left, *left_range)

                else:
                    left_size = 0

                if right_child is not None:
                    right_range = (low + left_size, high)

                    if self._in_range(changed, right_range):
                        _update(right, *right_range)

                tree[tree_index] = self._merge(tree_index)

        _update(0, 0, self._size - 1)

    def insert(self, index: int, value: _T) -> None:
        """Insert a new value to the array

        Asymptotic in time complexity: O(logH), with H is the height of the tree
        """
        if not self._in_range(index, (0, self._size)):
            message = f"Invalid insertion index {index}"
            raise ValueError(message)

        tree = self._tree
        if index == 0:
            def _insert_front(tree_index: int, current_value: Optional[_T]) -> None:
                left = 2 * tree_index + 1
                right = 2 * tree_index + 2
                tree[left] = (value, 1)
                tree[right] = None if current_value is None else (current_value, 1)
                tree[tree_index] = self._merge(tree_index)

            self._update(0, at_position=_insert_front)

        elif index == self._size:
            def _insert_back(tree_index: int, current_value: Optional[_T]) -> None:
                left = 2 * tree_index + 1
                right = 2 * tree_index + 2
                tree[left] = None if current_value is None else (current_value, 1)
                tree[right] = (value, 1)
                tree[tree_index] = self._merge(tree_index)

            self._update(self._size - 1, at_position=_insert_back)

        else:
            lca = self._find_lca(index - 1, index)
            left = tree[2 * lca + 1]
            right = tree[2 * lca + 2]

            def _update_left() -> None:
                def _insert(tree_index: int, current_value: Optional[_T]) -> None:
                    left = 2 * tree_index + 1
                    right = 2 * tree_index + 2
                    tree[left] = None if current_value is None else (current_value, 1)
                    tree[right] = (value, 1)
                    tree[tree_index] = self._merge(tree_index)

                self._update(index - 1, at_position=_insert)

            def _update_right() -> None:
                def _insert(tree_index: int, current_value: Optional[_T]) -> None:
                    left = 2 * tree_index + 1
                    right = 2 * tree_index + 2
                    tree[left] = (value, 1)
                    tree[right] = None if current_value is None else (current_value, 1)
                    tree[tree_index] = self._merge(tree_index)

                self._update(index, at_position=_insert)

            if left is None:
                _update_right()
            elif right is None:
                _update_left()
            elif left[1] < right[1]:
                _update_left()
            else:
                _update_right()

        self._size += 1

    def remove(self, index: int, /) -> None:
        """Remove an element at a specified index from the array

        Asymptotic in time complexity: O(logH), with H is the height of the tree
        """
        def _remove(tree_index: int, _: Optional[_T]) -> None:
            self._tree[tree_index] = None

        self._update(index, at_position=_remove)
        self._size -= 1

    def sum(self, query_low: int, query_high: int, /) -> _T:
        """Calculate the sum of an interval from the array

        Asymptotic in time complexity: O(logH), with H is the height of the tree
        """
        if query_low > query_high:
            query_low, query_high = query_high, query_low

        if not self._in_range(query_low, (0, self._size - 1)):
            message = f"Invalid query_low: {query_low}"
            raise ValueError(message)

        if not self._in_range(query_high, (0, self._size - 1)):
            message = f"Invalid query_high: {query_high}"
            raise ValueError(message)

        query_range = (query_low, query_high)
        tree = self._tree

        def _sum(tree_index: int, low: int, high: int) -> Optional[_T]:
            current = tree[tree_index]
            if low == high:
                if current is not None and self._in_range(low, query_range):
                    return current[0]
                else:
                    return None

            if query_low <= low and high <= query_high:
                return None if current is None else current[0]

            if high < query_low or query_high < low:
                return None

            left_index = 2 * tree_index + 1
            right_index = 2 * tree_index + 2
            left = tree[left_index]
            right = tree[right_index]

            result: Optional[_T] = None
            if left is not None:
                left_size = left[1]
                left_range = (low, low + left_size - 1)
                result = self.operator(result, _sum(left_index, *left_range))
            else:
                left_size = 0

            if right is not None:
                right_range = (low + left_size, high)
                result = self.operator(result, _sum(right_index, *right_range))

            return result

        result = _sum(0, 0, self._size - 1)
        assert result is not None
        return result

    def flatten(self) -> List[_T]:
        """Returns the underlying array that this tree represents"""
        array: List[Any] = [None] * self._size
        tree = self._tree

        def _flatten(tree_index: int, low: int, high: int) -> None:
            if low == high:
                current = tree[tree_index]
                assert current is not None
                array[low] = current[0]
            else:
                left_index = 2 * tree_index + 1
                right_index = 2 * tree_index + 2

                left = tree[left_index]
                right = tree[right_index]

                if left is not None:
                    left_size = left[1]
                    _flatten(left_index, low, low + left_size - 1)
                else:
                    left_size = 0

                if right is not None:
                    _flatten(right_index, low + left_size, high)

        _flatten(0, 0, self._size - 1)
        return array
