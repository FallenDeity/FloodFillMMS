from __future__ import annotations

import time

from utils.api import COLORS, Mouse
from utils.astar import AStar
from utils.bfs import BFS
from utils.dfs import DFS
from utils.floodfill import FloodFill


def floodfill():
    flood = FloodFill(mouse=Mouse())
    flood.log(f"Starting at {flood.current}")
    counter = 0
    while True:
        if counter == 6:
            break
        start = time.perf_counter()
        flood.run(debug=True)
        flood.log(f"Took {time.perf_counter() - start} seconds")
        for i in flood.flood:
            flood.log(i)
        flood.reset(manual=False)
        counter += 1
    flood.log(flood.paths)
    path = flood.paths[min(flood.paths.keys())]
    flood.log(f"Running optimized path of length {len(path)}")
    start = time.perf_counter()
    flood.reset(manual=True)
    for step in path:
        flood.move(step)
        flood.mouse.set_color(*step, COLORS.DARK_RED)

    flood.log(f"Took {time.perf_counter() - start} seconds")


def astar():
    astar = AStar(mouse=Mouse())
    astar.log(f"Starting at {astar.current}")
    start = time.perf_counter()
    path = astar.run(debug=True)
    astar.log(f"Took {time.perf_counter() - start} seconds")
    astar.log(f"Running optimized path of length {len(path)}")
    astar.reset()
    start = time.perf_counter()
    for point in path:
        astar.move(point)
        astar.mouse.set_color(*point, COLORS.DARK_BLUE)
    astar.log(f"Took {time.perf_counter() - start} seconds")


def bfs():
    bfs = BFS(mouse=Mouse())
    bfs.log(f"Starting at {bfs.current}")
    start = time.perf_counter()
    path = bfs.run(debug=True)
    bfs.log(f"Took {time.perf_counter() - start} seconds")
    bfs.log(f"Running optimized path of length {len(path)}")
    bfs.reset()
    start = time.perf_counter()
    for point in path:
        bfs.move(point)
        bfs.mouse.set_color(*point, COLORS.DARK_GREEN)
    bfs.log(f"Took {time.perf_counter() - start} seconds")


def dfs():
    dfs = DFS(mouse=Mouse())
    dfs.log(f"Starting at {dfs.current}")
    start = time.perf_counter()
    path = dfs.run(debug=True)
    dfs.log(f"Took {time.perf_counter() - start} seconds")
    dfs.log(f"Running optimized path of length {len(path)}")
    dfs.reset()
    start = time.perf_counter()
    for point in path:
        dfs.move(point)
        dfs.mouse.set_color(*point, COLORS.DARK_CYAN)
    dfs.log(f"Took {time.perf_counter() - start} seconds")


def reset():
    mouse = Mouse()
    mouse.ack_reset()
    mouse.clear_all_color()


def main():
    floodfill()
    # reset()
    # astar()


if __name__ == "__main__":
    main()
