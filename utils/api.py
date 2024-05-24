from __future__ import annotations

import dataclasses
import enum
import sys
import typing

from . import Drive

__all__: typing.Tuple[str, ...] = (
    "COLORS",
    "COMMANDS",
    "Command",
    "MouseCrashedError",
    "Mouse",
    "DIRECTIONS",
)


class DIRECTIONS(enum.IntEnum):
    EAST = 1
    SOUTH = 2
    WEST = 3
    NORTH = 4

    @staticmethod
    def from_int(value: int) -> str:
        return DIRECTIONS(value).name.lower()


class COLORS(enum.Enum):
    BLACK = "k"
    BLUE = "b"
    CYAN = "c"
    GRAY = "a"
    GREEN = "g"
    ORANGE = "o"
    RED = "r"
    WHITE = "w"
    YELLOW = "y"
    DARK_BLUE = "B"
    DARK_CYAN = "C"
    DARK_GRAY = "A"
    DARK_GREEN = "G"
    DARK_RED = "R"
    DARK_YELLOW = "Y"


class COMMANDS(enum.Enum):
    MAZEWIDTH = "mazeWidth"
    MAZEHEIGHT = "mazeHeight"
    WALLFRONT = "wallFront"
    WALLRIGHT = "wallRight"
    WALLLEFT = "wallLeft"
    MOVEFORWARD = "moveForward"
    TURNRIGHT = "turnRight"
    TURNLEFT = "turnLeft"
    SETWALL = "setWall"
    CLEARWALL = "clearWall"
    SETCOLOR = "setColor"
    CLEARCOLOR = "clearColor"
    CLEARALLCOLOR = "clearAllColor"
    SETTEXT = "setText"
    CLEARTEXT = "clearText"
    CLEARALLTEXT = "clearAllText"
    WASRESET = "wasReset"
    ACKRESET = "ackReset"


@dataclasses.dataclass
class Command:
    name: COMMANDS
    args: tuple[typing.Any, ...]
    return_type: typing.Optional[type[typing.Any]] = None

    @property
    def line(self) -> str:
        return " ".join(
            map(
                str,
                (self.name.value, *self.args),
            )
        )


class MouseCrashedError(Exception):
    def __init__(self, message: str = "Your mouse crashed.") -> None:
        self.message = message

    def __str__(self) -> str:
        return self.message


@typing.final
class Mouse(Drive):
    @staticmethod
    def command(command: Command) -> typing.Any:
        sys.stdout.write(f"{command.line}\n")
        sys.stdout.flush()
        if command.return_type:
            response = sys.stdin.readline().strip()
            if command.return_type == bool:
                return response == str(True).lower()
            return command.return_type(response)
        return None

    @property
    def maze_width(self) -> int:
        return self.command(Command(COMMANDS.MAZEWIDTH, (), int))

    @property
    def maze_height(self) -> int:
        return self.command(Command(COMMANDS.MAZEHEIGHT, (), int))

    @property
    def wall_front(self) -> bool:
        return self.command(Command(COMMANDS.WALLFRONT, (), bool))

    @property
    def wall_right(self) -> bool:
        return self.command(Command(COMMANDS.WALLRIGHT, (), bool))

    @property
    def wall_left(self) -> bool:
        return self.command(Command(COMMANDS.WALLLEFT, (), bool))

    @property
    def was_reset(self) -> bool:
        return self.command(Command(COMMANDS.WASRESET, (), bool))

    def move_forward(self, distance: typing.Optional[int] = None) -> None:
        args = (distance,) if distance else ()
        response = self.command(Command(COMMANDS.MOVEFORWARD, args, str))
        if response == "crash":
            raise MouseCrashedError()

    def turn_right(self) -> None:
        self.command(Command(COMMANDS.TURNRIGHT, (), str))

    def turn_left(self) -> None:
        self.command(Command(COMMANDS.TURNLEFT, (), str))

    def set_wall(self, x: int, y: int, direction: DIRECTIONS) -> None:
        self.command(Command(COMMANDS.SETWALL, (x, y, DIRECTIONS.from_int(direction)[0])))

    def clear_wall(self, x: int, y: int, direction: DIRECTIONS) -> None:
        self.command(Command(COMMANDS.CLEARWALL, (x, y, DIRECTIONS.from_int(direction)[0])))

    def clear_color(self, x: int, y: int) -> None:
        self.command(Command(COMMANDS.CLEARCOLOR, (x, y)))

    def clear_all_color(self) -> None:
        self.command(Command(COMMANDS.CLEARALLCOLOR, ()))

    def set_color(self, x: int, y: int, color: COLORS) -> None:
        self.command(Command(COMMANDS.SETCOLOR, (x, y, color.value)))

    def set_text(self, x: int, y: int, text: str) -> None:
        self.command(Command(COMMANDS.SETTEXT, (x, y, text)))

    def clear_text(self, x: int, y: int) -> None:
        self.command(Command(COMMANDS.CLEARTEXT, (x, y)))

    def clear_all_text(self) -> None:
        self.command(Command(COMMANDS.CLEARALLTEXT, ()))

    def ack_reset(self) -> None:
        self.command(Command(COMMANDS.ACKRESET, (), str))
