#!/usr/bin/env python

from robotics.motion2D import is_collision_free, \
    create_box, sample_box, is_region, inf_sequence, sample, draw_solution

from stripstream.fts.constraint import Eq, ConType
from stripstream.fts.variable import VarType, Par, Var, X, nX
from stripstream.fts.clause import Clause
from stripstream.fts.sampler import Sampler
from stripstream.fts.problem import FTSProblem
from stripstream.fts.stripstream_conversion import constraint_to_stripstream
from stripstream.fts.utils import convert_plan, rename_variables

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward


def create_problem(goal, obstacles=(), distance=.25, digits=3):
    """
    Creates a Probabilistic Roadmap (PRM) motion planning problem.

    :return: a :class:`.FTSProblem`
    """

    R_Q = 'R_Q'
    CONF, REGION = VarType(), VarType(domain=[goal] if is_region(goal) else [])
    Q, R = Par(CONF), Par(REGION)

    IsEdge = ConType([CONF, CONF], test=lambda q1,
                     q2: is_collision_free((q1, q2), obstacles))  # CFree
    Contained = ConType([CONF, REGION])

    rename_variables(locals())  # Trick to make debugging easier

    state_vars = [Var(R_Q, CONF)]
    control_vars = []

    ##########

    clauses = [
        Clause([IsEdge(X[R_Q], nX[R_Q])], name='move'),
    ]

    ##########

    samplers = [
        Sampler([Q], gen=lambda: ([(sample(),)]
                                  for _ in inf_sequence()), inputs=[]),
        Sampler([Contained(Q, R)], gen=lambda r: ([(sample_box(r),)]
                                                  for _ in inf_sequence()), inputs=[R]),
    ]

    ##########

    initial_state = [Eq(X[R_Q], (0, 0))]
    goal_constraints = []
    if is_region(goal):
        goal_constraints.append(Contained(X[R_Q], goal))
    else:
        goal_constraints.append(Eq(X[R_Q], goal))

    return FTSProblem(state_vars, control_vars, clauses, samplers,
                      initial_state, goal_constraints)

##################################################


def main():
    """
    Creates and solves the 2D motion planning problem.
    """

    obstacles = [
        create_box((.5, .5), .2, .2)
    ]
    goal = create_box((.9, .9), .2, .2)

    constraint_problem = create_problem(goal, obstacles)
    print
    print constraint_problem

    stream_problem = constraint_to_stripstream(constraint_problem)
    print
    print stream_problem

    search = get_fast_downward('astar')  # dijkstra | astar
    plan, _ = incremental_planner(
        stream_problem, search=search, frequency=25, waves=False)
    plan = convert_plan(plan)
    print 'Plan:', plan

    draw_solution(plan, goal, obstacles)
    raw_input('Finish?')

if __name__ == '__main__':
    main()
