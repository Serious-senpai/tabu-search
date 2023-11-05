# tabu-search
[![Lint](https://github.com/Serious-senpai/tabu-search/actions/workflows/lint.yml/badge.svg)](https://github.com/Serious-senpai/tabu-search/actions/workflows/lint.yml)
[![Tests](https://github.com/Serious-senpai/tabu-search/actions/workflows/tests.yml/badge.svg)](https://github.com/Serious-senpai/tabu-search/actions/workflows/tests.yml)

Tabu search technique for solving optimization problems. TSP problems are taken from [TSPLIB](http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95).

## Setup
- It is highly recommended to create a virtual environment by running `python -m venv .venv`
- Install dependencies for running the optimizers
```
$ pip install -r requirements.txt
```

## Developing
- Install dependencies for development
```
$ pip install -r dev-requirements.txt
```
- Codes must be formatted using [autopep8](https://pypi.org/project/autopep8). Run `autopep8` at the root of the repository
```
$ autopep8 -aaair .
```
- Codes must pass the [mypy](https://pypi.org/project/mypy) type-checking.
```
$ mypy .
```
