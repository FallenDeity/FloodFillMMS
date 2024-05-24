import abc
import typing as t

if t.TYPE_CHECKING:
    from utils.api import COLORS


class Drive(abc.ABC):
    @property
    @abc.abstractmethod
    def maze_width(self) -> int:
        ...

    @property
    @abc.abstractmethod
    def maze_height(self) -> int:
        ...

    @property
    @abc.abstractmethod
    def wall_front(self) -> bool:
        ...

    @property
    @abc.abstractmethod
    def wall_right(self) -> bool:
        ...

    @property
    @abc.abstractmethod
    def wall_left(self) -> bool:
        ...

    @abc.abstractmethod
    def move_forward(self, distance: t.Optional[int] = None) -> None:
        ...

    @abc.abstractmethod
    def turn_right(self) -> None:
        ...

    @abc.abstractmethod
    def turn_left(self) -> None:
        ...

    @abc.abstractmethod
    def ack_reset(self) -> None:
        ...

    @abc.abstractmethod
    def set_color(self, x: int, y: int, color: "COLORS") -> None:
        ...
