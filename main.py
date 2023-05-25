from __future__ import annotations

import time

from utils.api import Mouse
from utils.floodfill import FloodFill


def main():
    flood = FloodFill(mouse=Mouse())
    counter = 0
    while True:
        if counter == 5:
            break
        start = time.perf_counter()
        flood.run()
        flood.log(f"Took {time.perf_counter() - start} seconds")
        flood.reset()
        counter += 1
    path = flood.paths[min(flood.paths.keys())]
    flood.log(f"Running optimized path of length {len(path)}")
    start = time.perf_counter()
    for step in path:
        flood.move(step)
    flood.log(f"Took {time.perf_counter() - start} seconds")


if __name__ == "__main__":
    main()
