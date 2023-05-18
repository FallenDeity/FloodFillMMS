import abc
import typing as t


class Drive(abc.ABC):
    @abc.abstractproperty
    def maze_width(self) -> int:
        ...

    @abc.abstractproperty
    def maze_height(self) -> int:
        ...

    @abc.abstractproperty
    def wall_front(self) -> bool:
        ...

    @abc.abstractproperty
    def wall_right(self) -> bool:
        ...

    @abc.abstractproperty
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
