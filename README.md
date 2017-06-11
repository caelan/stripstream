# stripstream

STRIPStream is a Python library containing both a language for modeling planning problems in infinite domains and algorithms for solving these problems.

STRIPStream is still in development. Additional documentation and examples will be added shortly. In particular, several robotics examples using OpenRAVE and an in-house, standalone Python robotics simulator are on the way.

## Documentation

https://caelan.github.io/stripstream/

http://web.mit.edu/caelan/www/research/stripstream/

## Optional Dependencies

Tkinter - https://wiki.python.org/moin/TkInter

OpenRAVE - http://openrave.org/

## Search Subroutines

STRIPStream optionally supports using arbitrary PDDL planners to implement the ```search``` subroutine. The following planners are currently supported. Additional planners will be added in the future. Contact me if you are interested in supporting a new planner. The default ```search``` subroutine implementation uses a slow Python Breadth-First Search (BFS), so installing one of these planners is highly recommended.

### FastDownward

Follow these instructions to install FastDownward: 

http://www.fast-downward.org/ObtainingAndRunningFastDownward

Once installed, set the environment variable FD_PATH to be the build of the FastDownward installation.
It should look similar to ```.../FastDownward/builds/release32/``` depending on directory and chosen release.

### FastForward

FastForward is available here:

https://fai.cs.uni-saarland.de/hoffmann/ff.html

I use FF-v2.3 and FF-X (supports derived predicates). Once installed, set the environment variable FF_PATH to be the root of the FastForward installation.
It should look similar to ```.../FF-v2.3/``` or ```.../FF-X/```.

## Tutorial

The following tutorial implements a countable task and motion planning (TAMP) problem:

https://github.mit.edu/pages/caelan/stripstream/tutorial.html

You can run this tutorial using the following command:

```python -m scripts.run_tutorial```

## Examples

The following scripts contain example STRIPStream formulations:

```
python -m scripts.run_tutorial
python -m scripts.run_pddl_tutorial
python -m scripts.blocksworld
python -m scripts.run_countable_tamp [--search SEARCH]  [-focus] [-viewer] [-display]
python -m scripts.run_continuous_tamp [-fd] [-focus] [-viewer] [-display]
```

## Testing

The following command executes tests that do not require FastDownward:

```python -m unittest tests.test_countable_bfs```

The following command executes tests that require FastDownward:

```python -m unittest tests.test_countable_fd```

The following command executes all tests:

```python -m unittest discover -s tests```

Additional tests will be added in the future.

## Publications

STRIPS Planning in Infinite Domains - https://arxiv.org/abs/1701.00287

## Citation

Caelan R. Garrett, Tomas Lozano-Perez, Leslie P. Kaelbling. STRIPS Planning in Infinite Domains, ICAPS Workshop on Planning and Robotics (PlanRob), 2017.
