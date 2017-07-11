#!/usr/bin/env python2

from stripstream.fts.constraint import Eq, ConType
from stripstream.fts.variable import VarType, Par, Var, X, U, nX
from stripstream.fts.clause import Clause
from stripstream.fts.sampler import Sampler
from stripstream.fts.problem import FTSProblem
from stripstream.fts.stripstream_conversion import constraint_to_stripstream
from stripstream.fts.utils import convert_plan, rename_variables

from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.algorithms.incremental.incremental_planner import incremental_planner


def create_problem():
    """
    Creates the 1D task and motion planning FTSProblem problem.

    :return: a :class:`.FTSProblem`
    """

    blocks = ['block%i' % i for i in range(2)]
    num_poses = pow(10, 10)

    initial_config = 0  # the initial robot configuration is 0
    initial_poses = {block: i for i, block in enumerate(
        blocks)}  # the initial pose for block i is i

    # the goal pose for block i is i+1
    goal_poses = {block: i + 1 for i, block in enumerate(blocks)}

    ####################

    R_Q, B_P, B_H = 'R_Q', 'B_P', 'B_H'
    R_T = 'R_T'

    CONF = VarType()
    BOOL = VarType(domain=[True, False])
    POSE = VarType()
    BLOCK = VarType(domain=blocks)
    TRAJ = VarType()

    B, Q, P = Par(BLOCK), Par(CONF), Par(POSE)
    T, Q2 = Par(TRAJ), Par(CONF)

    LegalKin = ConType([POSE, CONF])
    CollisionFree = ConType([POSE, POSE], test=lambda p1,
                            p2: None in (p1, p2) or p1 != p2)
    Motion = ConType([CONF, TRAJ, CONF])

    rename_variables(locals())  # Trick to make debugging easier

    state_vars = [Var(R_Q, CONF), Var(B_P, POSE, args=[BLOCK]),
                  Var(B_H, BOOL, args=[BLOCK])]
    control_vars = [Var(R_T, TRAJ)]

    ##########

    transition = [
        Clause([LegalKin(X[B_P, B], X[R_Q]), Eq(nX[B_P, B], None), Eq(X[B_H, B], False), Eq(nX[B_H, B], True)] +
               [Eq(X[B_H, block], False) for block in blocks], name='pick'),

        Clause([LegalKin(nX[B_P, B], X[R_Q]), Eq(X[B_P, B], None), Eq(X[B_H, B], True), Eq(nX[B_H, B], False)] +
               [CollisionFree(X[B_P, block], nX[B_P, B]) for block in blocks], name='place'),

        Clause([Motion(X[R_Q], U[R_T], nX[R_Q])], name='move'),
    ]

    ##########

    samplers = [
        Sampler([P], gen=lambda: ([(p,)]
                                  for p in xrange(num_poses)), inputs=[]),
        Sampler([LegalKin(P, Q)], gen=lambda p: [[(p,)]]
                if p is not None else [], inputs=[P]),
        Sampler([Motion(Q, T, Q2)], gen=lambda q1,
                q2: [[((q1, q2),)]], inputs=[Q, Q2]),
    ]

    ##########

    initial_state = [Eq(X[R_Q], initial_config)] + \
                    [Eq(X[B_H, block], False) for block in blocks] + \
                    [Eq(X[B_P, block], pose)
                        for block, pose in initial_poses.iteritems()]
    goal_constraints = [Eq(X[B_P, block], pose)
                        for block, pose in goal_poses.iteritems()]

    return FTSProblem(state_vars, control_vars, transition, samplers,
                      initial_state, goal_constraints)

##################################################


def main():
    """
    Creates and solves the 1D task and motion planning FTSProblem problem.
    """

    constraint_problem = create_problem()
    print
    print constraint_problem

    stream_problem = constraint_to_stripstream(constraint_problem)
    print
    print stream_problem

    # 'dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
    search_fn = get_fast_downward('eager')

    plan, _ = incremental_planner(stream_problem, search=search_fn)
    print
    print 'Plan:', convert_plan(plan)

if __name__ == '__main__':
    main()
