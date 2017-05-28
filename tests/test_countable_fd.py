import unittest

from tests.test_countable_bfs import TestCountable
from stripstream.algorithms.utils import FAST_DOWNWARD, INCREMENTAL, FOCUSED


class TestCountableIncrementalFD(TestCountable, unittest.TestCase):
    planner = INCREMENTAL
    search = FAST_DOWNWARD


class TestCountableFocusedFD(TestCountable, unittest.TestCase):
    planner = FOCUSED
    search = FAST_DOWNWARD

if __name__ == '__main__':
    unittest.main()
