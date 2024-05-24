from __future__ import annotations

import time

from utils.api import COLORS, Mouse
from utils.astar import AStar
from utils.floodfill import FloodFill


def floodfill():
    flood = FloodFill(mouse=Mouse())
    print(f"Starting at {flood.current}")
    counter = 0
    while True:
        if counter == 5:
            break
        start = time.perf_counter()
        flood.run(debug=True)
        flood.log(f"Took {time.perf_counter() - start} seconds")
        flood.reset(manual=False)
        counter += 1
    path = flood.paths[min(flood.paths.keys())]
    flood.log(f"Running optimized path of length {len(path)}")
    start = time.perf_counter()
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
        astar.mouse.set_color(*point, COLORS.DARK_RED)
    astar.log(f"Took {time.perf_counter() - start} seconds")


def reset():
    mouse = Mouse()
    mouse.ack_reset()
    mouse.clear_all_color()


def main():
    floodfill()
    reset()
    astar()


if __name__ == "__main__":
    main()
