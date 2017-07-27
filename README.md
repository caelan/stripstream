# stripstream

STRIPStream is a Python library containing languages for modeling planning problems in infinite domains and algorithms for solving these problems. 

STRIPStream includes two representation languages: the STRIPStream language and the Factored Transition System (FTS) Language. The STRIPStream language is a programmatic extension of PDDL (Planning Domain Definition Language). STRIPStream problem formluations are in ```scripts/```. 

The FTS Language is a way of describing transition systems factorable into state and control variables as well as constrains on these variabes. FTS problem formluations are in ```fts_scripts/```. FTS problems are automatically compiled into STRIPStream problem instances and solved using the same set of algorithms. I recommend using STRIPStream over FTS when modeling problems as it is more flexible. 

STRIPStream is still in development. Additional documentation and examples will be added shortly. In particular, several robotics examples using OpenRAVE and an in-house, standalone Python robotics simulator are on the way.

## Documentation

https://caelan.github.io/stripstream/

http://web.mit.edu/caelan/www/research/stripstream/

## Optional Dependencies

NumPy - http://www.numpy.org/

Tkinter - https://wiki.python.org/moin/TkInter

OpenRAVE - http://openrave.org/, https://github.com/rdiankov/openrave

PyR2 - https://github.com/caelan/PyR2

### Virtual Machine

I've provided a VMWare virtual machine (VM) that has STRIPStream, FastDownward, NumPy, Tkinter, and OpenRAVE installed on it. Remember to periodically pull STRIPStream and the other libraries to obtain the lastest version. The VM username is ss-or, and the password is also ss-or. 

https://www.dropbox.com/sh/g7dbz3sdiv8zknn/AAD705L2YbB0ortsKPNY7WiZa?dl=0

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

## Pure Python Examples

The following scripts contain example STRIPStream formulations that soley use Python:

STRIPStream
```
python -m scripts.run_tutorial
python -m scripts.run_pddl_tutorial
python -m scripts.run_blocksworld
python -m scripts.run_countable_tamp [--search SEARCH]  [-focus] [-viewer] [-display]
python -m scripts.run_continuous_tamp [-fd] [-focus] [-viewer] [-display]
```

FTS
```
python -m fts_scripts.run_tutorial
python -m fts_scripts.run_prm
```

## PyR2 Examples

Install the following repositories and add them to your PYTHONPATH environment variable.

https://github.com/caelan/PyR2
https://github.com/caelan/motion-planners

```
python -m scripts.run_pyr2_fixed_base
```

## OpenRAVE Examples

The following scripts contain example STRIPStream formulations using OpenRAVE:

STRIPStream
```
python -m scripts.run_openrave_tamp_fixed_base [-viewer]
```

FTS
```
python -m fts_scripts.run_openrave_tamp_fixed_base [-viewer]
```

### OpenRAVE Installation

OpenRAVE can be difficult to install. OpenRAVE only reliably supports Ubuntu. See the following blog posts for Ubuntu intallation instructions. I recommend building from master in order to take advantage of the latest improvements.

Installing OpenRAVE on Ubuntu 14.04 - https://scaron.info/teaching/installing-openrave-on-ubuntu-14.04.html

Installing OpenRAVE on Ubuntu 16.04 - https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html

Installing OpenRAVE in Ubuntu 14.04 (Trusty) - http://fsuarez6.github.io/blog/openrave-trusty/

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

Sample-Based Methods for Factored Task and Motion Planning - http://web.mit.edu/caelan/www/publications/rss2017.pdf

## Citations

Caelan R. Garrett, Tomás Lozano-Pérez, Leslie P. Kaelbling. STRIPS Planning in Infinite Domains, ICAPS Workshop on Planning and Robotics (PlanRob), 2017.

Caelan R. Garrett, Tomás Lozano-Pérez, Leslie P. Kaelbling. Sample-Based Methods for Factored Task and Motion Planning, Robotics: Science and Systems (RSS), 2017.

