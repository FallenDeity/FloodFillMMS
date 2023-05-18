from __future__ import annotations

import copy
import sys
import typing
from collections import deque
from functools import lru_cache

from utils.api import DIRECTIONS

if typing.TYPE_CHECKING:
    from . import Drive


__all__: typing.Tuple[str, ...] = ("FloodFill",)
PointType = typing.Tuple[int, int]


class FloodFill:
    __slots__: typing.Tuple[str, ...] = (
        "mouse",
        "w",
        "h",
        "orient",
        "queue",
        "current",
        "cells",
        "flood",
        "paths",
        "walls",
    )
    mapping = {DIRECTIONS.EAST: (0, 1), DIRECTIONS.SOUTH: (1, 0), DIRECTIONS.WEST: (0, -1), DIRECTIONS.NORTH: (-1, 0)}

    def __init__(self, mouse: "Drive") -> None:
        self.mouse = mouse
        self.w, self.h = mouse.maze_width, mouse.maze_height
        self.orient = DIRECTIONS.EAST
        self.queue: typing.Deque[PointType] = deque()
        self.current = (0, 0)
        self.cells = [[0 for _ in range(self.w)] for _ in range(self.h)]
        self.flood = self.generate_flood_matrix()
        self.paths: typing.Dict[int, typing.List[PointType]] = {}
        self.walls: typing.Dict[PointType, typing.List[PointType]] = {}

    @staticmethod
    def log(*args: typing.Any) -> None:
        """
        The log function is a simple wrapper around the stderr stream.
        It takes any number of arguments and prints them to stderr, separated by spaces.

        :param args: typing.Any: Pass in a variable number of arguments
        :return: None, so we can't use it in an expression
        """
        sys.stderr.write(f"{' '.join(map(str, args))}\n")

    def generate_flood_matrix(self) -> typing.List[typing.List[int]]:
        """
        The generate_flood_matrix function is used to generate a matrix of values that will be used to determine the
        flood level for each tile in the map. The function takes no arguments, and returns a list of lists containing
        integers.

        :return: A 2d array of integers, where each integer represents the number of tiles
        """
        midpoint = (self.w - 1) / 2
        matrix: typing.List[typing.List[int]] = []
        for y in range(self.h):
            row: typing.List[int] = []
            for x in range(self.w):
                val = midpoint - abs(x - midpoint) + max(0, int(y - midpoint))
                row.append(int(val))
            row = row[: int(midpoint + 1)][::-1]
            matrix.append(row + copy.deepcopy(row[::-1]))
        matrix = matrix[::-1][: int(midpoint + 1)]
        return matrix + copy.deepcopy(matrix[::-1])

    @lru_cache(maxsize=None)
    def neighbors(self, cell: PointType) -> typing.List[PointType]:
        """
        The neighbors function takes a cell and returns the cells surrounding it.

        :param cell: PointType: Define the cell that is being passed into the function
        :return: A list of the surrounding cells
        """
        surrounding: typing.List[PointType] = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                x, y = cell[0] + i, cell[1] + j
                if 0 <= x < self.w and 0 <= y < self.h and not all((i, j)) and any((i, j)):
                    surrounding.append((x, y))
        return surrounding

    def update_walls(self) -> None:
        """
        The update_walls function is used to update the walls of the maze.
        It takes in a mouse object and uses its wall_left, wall_right, and wall_front attributes to determine which
        walls are present. The function then adds these walls into a set called 'walls'. The function then checks
        if any of these new walls are already in the current cell's list of adjacent cells and adds them if they aren't.

        :return: None
        """
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

    def floodfill_all(self, cell: PointType) -> None:
        """
        The floodfill_all function is a recursive function that takes in a cell and
        assigns it the minimum value of its neighbors plus one. It then adds all of
        its neighbors to the queue, which will be processed by subsequent calls to
        floodfill_all. This process continues until there are no more cells left in
        the queue.

        :param cell: PointType: Specify the cell to be flooded
        :return: None
        """
        self.queue.append(cell)
        while self.queue:
            cell = self.queue.pop()
            walls = self.walls.get(cell, [])
            neighbors = [i for i in self.neighbors(cell) if i not in walls]
            n_vals = [self.flood[i[0]][i[1]] for i in neighbors]
            if self.flood[cell[0]][cell[1]] <= (m := min(n_vals)):
                self.flood[cell[0]][cell[1]] = m + 1
                self.queue.extend(neighbors)

    def find_cell(
        self,
        cell: PointType,
        ignore: typing.Optional[typing.Set[PointType]] = None,
    ) -> PointType:
        """
        The find_cell function is used to find the next cell in the maze.
        It takes two arguments: a cell and an optional set of cells to ignore.
        The function first finds all neighbors of the current cell that are not in
        the set of ignored cells, then it finds which neighbor has the lowest flood value,
        and finally it checks if that neighbor is a wall or not. If it's not a wall, then
        it returns that neighbor as its next_cell; otherwise, it adds this new 'wall' to its list
        of walls for this particular current cell and calls itself again with these new parameters.&quot;

        :param cell: PointType: Specify the cell that we want to find the next cell from
        :param ignore: typing.Optional[typing.Set[PointType]]: Ignore the cells that have already been visited
        :param : Find the next cell to move to
        :return: The next cell that the robot should move to
        """
        ignore = ignore or set()
        neighbors = [i for i in self.neighbors(self.current) if i not in ignore]
        n_vals = [self.flood[i[0]][i[1]] for i in neighbors]
        indices = [i for i, v in enumerate(n_vals) if v == min(n_vals)]
        orient = self.mapping[self.orient]
        check = [i for i in indices if (neighbors[i][0] - cell[0], neighbors[i][1] - cell[1]) == orient]
        next_cell = neighbors[check[0] if check else indices[0]]
        if next_cell not in self.walls.get(cell, []):
            if self.flood[cell[0]][cell[1]] <= self.flood[next_cell[0]][next_cell[1]]:
                self.floodfill_all(cell)
            return next_cell
        self.floodfill_all(cell)
        ignore.add(next_cell)
        return self.find_cell(cell, ignore)

    def move(self, cell: PointType) -> None:
        """
        The move function takes a cell as an argument and moves the mouse to that cell.

        :param cell: PointType: Determine the next cell that the mouse will move to
        :return: None
        """
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
    def cut_redundant_steps(steps: typing.List[PointType]) -> typing.List[PointType]:
        """
        The cut_redundant_steps function takes a list of steps and returns a new list with redundant steps removed.

        :param steps: typing.List[PointType]: Define the list of steps that will be passed into the function
        :return: A list of points that are the same as the input steps, but with redundant points removed
        """
        new_steps: typing.List[PointType] = []
        while steps:
            current = steps.pop(0)
            if current in steps:
                steps = steps[len(steps) - steps[::-1].index(current) :]
            new_steps.append(current)
        return new_steps

    def run(self) -> None:
        """
        The run function is the main function of the algorithm. It starts by setting
        the color and text of the current cell to green and &quot;Mickey&quot; respectively, then
        it logs that it's starting. Then, while there are still cells in flood fill, it
        updates its walls (which updates its flood fill), finds a new cell to move to,
        moves there (and adds that step to steps), then repeats until all cells have been
        visited. Once all cells have been visited, it logs how many steps were taken and
        then optimizes those steps using cut_redundant_steps().

        :return: None
        """
        self.log("Starting...")
        steps: typing.List[PointType] = []
        while self.flood[self.current[0]][self.current[1]]:
            self.update_walls()
            cell = self.find_cell(self.current)
            self.move(cell)
            steps.append(cell)
        self.log(f"Done! {len(steps)} steps taken.")
        steps = self.cut_redundant_steps(steps)
        self.log(f"Optimized to {len(steps)} steps.")
        self.paths[len(steps)] = steps

    def reset(self) -> None:
        """
        The reset function is used to reset the state of the environment.
        It should be called at the beginning of each episode, and it will return a new observation.

        :return: None
        """
        if self.paths:
            self.log("Moving back to start...")
            steps = self.paths[min(self.paths)]
            print(steps)
            for i in steps[-2::-1] + [(0, 0)]:
                self.move(i)
            self.mouse.turn_left()
            self.mouse.turn_left()
            self.log("Moved back to start!")
        self.orient = DIRECTIONS.EAST
        self.current = (0, 0)
