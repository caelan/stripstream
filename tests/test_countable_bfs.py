import unittest

# python -m unittest tests.test_examples
# python -m unittest discover -s tests # NOTE - automatically applies all tests that match 'test*.py'
# https://docs.python.org/2/library/unittest.html

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.simple_focused import simple_focused
from stripstream.pddl.examples.countable_tamp.countable_tamp_utils import get_grasp_problem, get_distract_problem, \
    get_invert_problem, get_shift_problem
from stripstream.pddl.examples.countable_tamp.countable_tamp import compile_problem
from stripstream.algorithms.search.bfs import get_bfs
from stripstream.algorithms.utils import BFS, FAST_DOWNWARD, INCREMENTAL, FOCUSED
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.utils import SEPARATOR

# TODO - additional tests
# TODO - test additional configurations of this planner
# TODO - test additional planners


class TestCountable():
    p = 3

    def run_test(self, problem_fn, planner, search):
        print SEPARATOR
        print problem_fn.__name__, self.planner, self.search
        tamp_problem = problem_fn(p=self.p)
        stream_problem = compile_problem(tamp_problem)
        print stream_problem
        if search == BFS:
            search_fn = get_bfs()
        elif search == FAST_DOWNWARD:
            # 'dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
            search_fn = get_fast_downward('eager')
        else:
            raise ValueError(search)
        if planner == INCREMENTAL:
            plan, _ = incremental_planner(
                stream_problem, search=search_fn, waves=True, frequency=1, verbose=False)
        elif planner == FOCUSED:
            plan, _ = simple_focused(stream_problem, search=search_fn,
                                     check_feasible=False, greedy=False, dfs=True, verbose=False)
        else:
            raise ValueError(planner)
        self.assertTrue(plan is not None)

    def test_grasp(self):  # TODO - is there a way to automate this further
        self.run_test(get_grasp_problem, self.planner, self.search)

    def test_distract(self):
        self.run_test(get_distract_problem, self.planner, self.search)

    def test_shift(self):
        self.run_test(get_shift_problem, self.planner, self.search)

    def test_invert(self):
        self.run_test(get_invert_problem, self.planner, self.search)

##################################################


class TestCountableIncrementalBFS(TestCountable, unittest.TestCase):
    planner = INCREMENTAL  # TODO - having trouble with bound functions
    search = BFS


class TestCountableFocusedBFS(TestCountable, unittest.TestCase):
    planner = FOCUSED
    search = BFS

if __name__ == '__main__':
    unittest.main()
