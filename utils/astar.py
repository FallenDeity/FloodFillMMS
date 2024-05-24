from __future__ import annotations

import dataclasses
import heapq
import itertools
import sys
import typing
from functools import lru_cache

from utils.api import COLORS, DIRECTIONS

if typing.TYPE_CHECKING:
    from . import Drive


__all__: typing.Tuple[str, ...] = ("AStar",)
PointType = typing.Tuple[int, int]


@dataclasses.dataclass(kw_only=True, eq=False, slots=True)
class Node:
    parent: typing.Optional[Node]
    position: PointType
    g: float
    h: float
    f: float

    def __lt__(self, other: Node) -> bool:
        return self.f < other.f

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Node):
            return False
        return self.position == other.position

    def __iter__(self) -> typing.Iterator[PointType]:
        current_node = self
        while current_node:
            yield current_node.position
            current_node = typing.cast("Node", current_node.parent)


class AStar:
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

    def heuristic(self, a: PointType, b: PointType) -> float:
        """
        The heuristic function is used to calculate the distance between two points.

        :param a: PointType: The first point
        :param b: PointType: The second point
        :return: int: The distance between the two points
        """
        import math

        heuristics: typing.Dict[str, typing.Callable[[PointType, PointType], float]] = {
            "manhattan": lambda x, y: abs(x[0] - y[0]) + abs(x[1] - y[1]),
            "euclidean": lambda x, y: 4 * ((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2) ** 0.5,
            "octile": lambda x, y: max(abs(x[0] - y[0]), abs(x[1] - y[1])),
            "chebyshev": lambda x, y: max(abs(x[0] - y[0]), abs(x[1] - y[1])),
            "diagonal": lambda x, y: abs(x[0] - y[0])
            + abs(x[1] - y[1])
            + (2**0.5 - 2) * min(abs(x[0] - y[0]), abs(x[1] - y[1])),
            "centroid": lambda x, y: math.sqrt((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2)
            + math.sqrt((x[0] - self.w // 2) ** 2 + (x[1] - self.h // 2) ** 2)
            + math.sqrt((y[0] - self.w // 2) ** 2 + (y[1] - self.h // 2) ** 2),
            "none": lambda x, y: 0,
        }
        # euclidean heuristic multiplier with weight 4 gives best results and centroid heuristic is the best
        return heuristics["euclidean"](a, b)

    @lru_cache(maxsize=None)
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

    def run(self, debug: bool = False) -> typing.List[PointType]:
        """
        The run function is used to run the A* algorithm. It creates 2 lists, open_list and closed_list. The open_list
        is a list of nodes that have been visited but not yet expanded. The closed_list is a list of nodes that have
        been visited and expanded. We start with the start node and add it to the open_list. We then loop through the
        open_list until it is empty. We pop the first node from the open_list and add it to the closed_list. We then
        check if the current node is the destination node. If it is, we return the path from the start node to the
        destination node. If it is not, we generate the children of the current node and add them to the open_list.
        We then loop through the children and calculate the f value of each child. If the child is in the closed_list,
        we skip it. If the child is in the open_list, we skip it. If the child is not in the open_list, we add it to the
        open_list. We then repeat the process until we find the destination node or the open_list is empty.
        """
        start_node = Node(parent=None, position=self.current, g=0, h=0, f=0)
        end_nodes = [Node(parent=None, position=pos, g=0, h=0, f=0) for pos in self.destination]
        last_node = start_node
        open_list: typing.List[Node] = []
        closed_list: typing.List[Node] = []
        heapq.heappush(open_list, start_node)

        while open_list:
            current_node = heapq.heappop(open_list)
            closed_list.append(current_node)

            to_path, from_path = list(current_node), list(last_node)
            i = None
            for i, point in enumerate(from_path):
                if point in to_path:
                    break

            i = i if i is not None else len(from_path)
            complete_path = from_path[: i + 1] + to_path[: to_path.index(from_path[i]) + 1][::-1]
            for point in complete_path:
                self.move(point)
                if debug:
                    self.mouse.set_color(*point, COLORS.DARK_YELLOW)
                self.update_walls()

            last_node = current_node

            if current_node in end_nodes:
                path: typing.List[PointType] = []
                while current_node:
                    path.append(current_node.position)
                    current_node = typing.cast(Node, current_node.parent)
                self.paths[len(path)] = path[::-1]
                return path[::-1]

            children: typing.List[Node] = []
            for new_position in self.neighbors(current_node.position):
                node_position = new_position
                end_node = min(end_nodes, key=lambda node: self.heuristic(node.position, node_position))
                if node_position in self.walls.get(current_node.position, []):
                    continue
                new_node = Node(
                    parent=current_node,
                    position=node_position,
                    g=current_node.g + 1,
                    h=self.heuristic(node_position, end_node.position),
                    f=0,
                )
                children.append(new_node)

            for child in children:
                if child in closed_list:
                    continue
                child.f = child.g + child.h
                if child in open_list:
                    continue
                heapq.heappush(open_list, child)
        return []

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

    @staticmethod
    def log(*args: typing.Any) -> None:
        """
        The log function is a simple wrapper around the stderr stream.
        It takes any number of arguments and prints them to stderr, separated by spaces.

        :param args: typing.Any: Pass in a variable number of arguments
        :return: None, so we can't use it in an expression
        """
        sys.stderr.write(f"{' '.join(map(str, args))}\n")

    @property
    def destination(self) -> typing.List[PointType]:
        """
        The destination property returns the destination of the mouse.

        :return: List[PointType]: The destination of the mouse
        """
        return list(
            itertools.product(range((self.w // 2) - 1, (self.w // 2) + 1), range((self.h // 2) - 1, (self.h // 2) + 1))
        )
