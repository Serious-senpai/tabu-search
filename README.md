# tsp-solver
[![Tests](https://github.com/Serious-senpai/tsp-solver/actions/workflows/tests.yml/badge.svg)](https://github.com/Serious-senpai/tsp-solver/actions/workflows/tests.yml)

Tabu search technique for solving TSP problems. Problems here are taken from [TSPLIB](http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95).

## Setup and run
- It is highly recommended to create a virtual environment by running `python -m venv .venv`
- Install dependencies
```
$ pip install -r requirements.txt
```
- View the help menu
```
$ python tsp.py --help
```

## Note
The distance between 2 points ($x_0, y_0$) and ($x_1, y_1$), by default, is calculated as the Manhattan distance $d = |x_1 - x_0| + |y_1 - y_0|$.
If you prefer Euclidean distance $d = \lfloor\sqrt{(x_1 - x_0) ^ 2 + (y_1 - y_0) ^ 2}\rfloor$, add the `-e` flag when running `tsp.py`. For example:
```
$ python tsp.py a280 -i 1000 -ve
```
Note that numerical roundings may vary according to different machines.
