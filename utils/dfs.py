from __future__ import annotations

import itertools
import sys
import typing
from collections import defaultdict, deque
from functools import lru_cache

from utils.api import COLORS, DIRECTIONS

if typing.TYPE_CHECKING:
    from . import Drive

__all__: typing.Tuple[str, ...] = ("DFS",)
PointType = typing.Tuple[int, int]


class DFS:
    __slots__ = (
        "mouse",
        "w",
        "h",
        "orient",
        "current",
        "paths",
        "walls",
    )
    mapping = {DIRECTIONS.EAST: (0, 1), DIRECTIONS.SOUTH: (1, 0), DIRECTIONS.WEST: (0, -1), DIRECTIONS.NORTH: (-1, 0)}

    def __init__(self, mouse: "Drive") -> None:
        self.mouse = mouse
        self.w, self.h = self.mouse.maze_width, self.mouse.maze_height
        self.orient = DIRECTIONS.EAST
        self.current = (0, 0)
        self.paths: typing.Dict[int, typing.List[PointType]] = {}
        self.walls: typing.Dict[PointType, typing.List[PointType]] = {}

    def update_walls(self) -> None:
        """
        The update_walls function is used to update the walls of the maze.
        It takes in a mouse object and uses its wall_left, wall_right, and wall_front attributes to determine which
        walls are present. The function then adds these walls into a set called 'walls'. The function then checks
        if any of these new walls are already in the current cell's list of adjacent cells and adds them if they aren't.

        :return: None
        """
        if self.current in self.walls:
            return
        lw, rw, fw = self.mouse.wall_left, self.mouse.wall_right, self.mouse.wall_front
        walls: typing.Set[DIRECTIONS] = set()
        if lw:
            walls.add(DIRECTIONS(self.orient - 1 if self.orient > 1 else 4))
        if fw:
            walls.add(self.orient)
        if rw:
            walls.add(DIRECTIONS(self.orient + 1 if self.orient < 4 else 1))
        c_walls = self.walls.get(self.current, [])
        for wall in walls:
            dx, dy = self.mapping[wall]
            x, y = self.current[0] + dx, self.current[1] + dy
            if (x, y) not in c_walls and 0 <= x < self.w and 0 <= y < self.h:
                self.walls.setdefault(self.current, []).append((x, y))

    def move(self, cell: PointType) -> None:
        """
        The move function takes a cell as an argument and moves the mouse to that cell.

        :param cell: PointType: Determine the next cell that the mouse will move to
        :return: None
        """
        self.log(f"Moving from {self.current} to {cell}")
        if cell == self.current or cell in self.walls.get(self.current, []):
            return
        diff = (
            cell[0] - self.current[0],
            cell[1] - self.current[1],
        )
        next_orient = [k for k, v in self.mapping.items() if v == diff][0]
        orient_diff = next_orient - self.orient
        check = DIRECTIONS(({orient_diff, orient_diff + 4} & set(range(1, 5))).pop())
        match check:
            case DIRECTIONS.NORTH:
                self.mouse.move_forward()
            case DIRECTIONS.EAST:
                self.mouse.turn_right()
                self.mouse.move_forward()
            case DIRECTIONS.WEST:
                self.mouse.turn_left()
                self.mouse.move_forward()
            case DIRECTIONS.SOUTH:
                self.mouse.turn_left()
                self.mouse.turn_left()
                self.mouse.move_forward()
        self.current = cell
        self.orient = DIRECTIONS(self.orient + orient_diff)

    @staticmethod
    def log(*args: typing.Any) -> None:
        """
        The log function is a simple wrapper around the stderr stream.
        It takes any number of arguments and prints them to stderr, separated by spaces.

        :param args: typing.Any: Pass in a variable number of arguments
        :return: None, so we can't use it in an expression
        """
        sys.stderr.write(f"{' '.join(map(str, args))}\n")

    def reset(self, manual: bool = True) -> None:
        """
        The reset function is used to reset the state of the environment.
        It should be called at the beginning of each episode, and it will return a new observation.

        :param manual: bool: Determine if the function should reset the mouse manually
        :return: None
        """
        if manual:
            self.mouse.ack_reset()
            self.current = (0, 0)
            self.orient = DIRECTIONS.EAST
            return
        if self.paths:
            self.log("Moving back to start...")
            steps = self.paths[min(self.paths)]
            for i in steps[-2::-1] + [(0, 0)]:
                self.move(i)
            self.log("Moved back to start!")

    @lru_cache
    def neighbors(self, cell: PointType) -> typing.List[PointType]:
        """
        The neighbors function takes a cell and returns the cells surrounding it.

        :param cell: PointType: Define the cell that is being passed into the function
        :param diagonals: bool: Determine if the function should return the diagonals
        :return: A list of the surrounding cells
        """
        surrounding: typing.List[PointType] = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                x, y = cell[0] + i, cell[1] + j
                if 0 <= x < self.w and 0 <= y < self.h and not all((i, j)) and any((i, j)):
                    surrounding.append((x, y))
        return surrounding

    def run(self, debug: bool = False) -> typing.List[PointType]:
        """
        The run function is used to run the DFS algorithm. It creates a visited set and a stack. The stack is a deque
        that contains the current cell. We then loop through the stack until it is empty. We pop the first cell from
        the stack and add it to the visited set. We then check if the current cell is the destination cell. If it is,
        we return the path from the start cell to the destination cell. If it is not, we generate the children of the
        current cell and add them to the stack. We then loop through the children and add them to the stack if they
        have not been visited yet. We then repeat the process until we find the destination cell or the stack is empty.

        :param debug: bool: Determine if the function should print debug information
        :return: List[PointType]: The path from the start cell to the destination cell
        """
        visited = set()
        stack = deque([self.current])
        parent: typing.DefaultDict[PointType, typing.Optional[PointType]] = defaultdict(lambda: None)
        while stack:
            current = stack.pop()
            visited.add(current)

            from_path: typing.List[PointType] = [self.current]
            check = parent[self.current]
            while check:
                from_path.append(check)
                check = parent[check]
            to_path: typing.List[PointType] = [current]
            check = parent[current]
            while check:
                to_path.append(check)
                check = parent[check]

            i = None
            for i, point in enumerate(from_path):
                if point in to_path:
                    break

            i = i if i is not None else len(from_path)
            complete_path = from_path[: i + 1] + to_path[: to_path.index(from_path[i]) + 1][::-1]
            for point in complete_path:
                self.move(point)
                self.update_walls()
                if debug:
                    self.mouse.set_color(*point, COLORS.DARK_YELLOW)

            if current in self.destination:
                path: typing.List[PointType] = []
                while current:
                    path.append(current)
                    current = typing.cast(PointType, parent[current])
                self.paths[len(path)] = path[::-1]
                return path[::-1]

            for neighbor in self.neighbors(current):
                if neighbor in visited or neighbor in self.walls.get(current, []):
                    continue
                parent[neighbor] = current
                stack.append(neighbor)
        return []

    @property
    def destination(self) -> typing.List[PointType]:
        """
        The destination property returns the destination of the mouse.

        :return: List[PointType]: The destination of the mouse
        """
        return list(
            itertools.product(range((self.w // 2) - 1, (self.w // 2) + 1), range((self.h // 2) - 1, (self.h // 2) + 1))
        )
