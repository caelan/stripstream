#!/usr/bin/env python

from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.utils import rename_easy
from itertools import product


def create_problem():
    """
    Creates the 1D task and motion planning STRIPStream problem.
    This models the same problem as :module:`.run_tutorial` but does so without using any streams.

    :return: a :class:`.STRIPStreamProblem`
    """

    num_blocks = 3
    blocks = ['block%i' % i for i in range(num_blocks)]
    num_poses = num_blocks + 1

    initial_config = 0  # the initial robot configuration is 0
    initial_poses = {block: i for i, block in enumerate(
        blocks)}  # the initial pose for block i is i

    # the goal pose for block i is i+1
    goal_poses = {block: i + 1 for i, block in enumerate(blocks)}

    ####################

    # Data types
    CONF, BLOCK, POSE = Type(), Type(), Type()

    # Fluent predicates
    AtConf = Pred(CONF)
    AtPose = Pred(BLOCK, POSE)
    HandEmpty = Pred()
    Holding = Pred(BLOCK)

    # Derived predicates
    Safe = Pred(BLOCK, BLOCK, POSE)

    # Static predicates
    LegalKin = Pred(POSE, CONF)
    CollisionFree = Pred(BLOCK, POSE, BLOCK, POSE)

    # Free parameters
    B1, B2 = Param(BLOCK), Param(BLOCK)
    P1, P2 = Param(POSE), Param(POSE)
    Q1, Q2 = Param(CONF), Param(CONF)

    rename_easy(locals())  # Trick to make debugging easier

    ####################

    actions = [
        Action(name='pick', parameters=[B1, P1, Q1],
               condition=And(AtPose(B1, P1), HandEmpty(),
                             AtConf(Q1), LegalKin(P1, Q1)),
               effect=And(Holding(B1), Not(AtPose(B1, P1)), Not(HandEmpty()))),

        Action(name='place', parameters=[B1, P1, Q1],
               condition=And(Holding(B1), AtConf(Q1), LegalKin(P1, Q1),
                             ForAll([B2], Or(Equal(B1, B2), Safe(B2, B1, P1)))),  # TODO - convert to finite blocks case?
               effect=And(AtPose(B1, P1), HandEmpty(), Not(Holding(B1)))),

        Action(name='move', parameters=[Q1, Q2],
               condition=AtConf(Q1),
               effect=And(AtConf(Q2), Not(AtConf(Q1)))),
    ]

    axioms = [
        Axiom(effect=Safe(B2, B1, P1),
              condition=Exists([P2], And(AtPose(B2, P2), CollisionFree(B1, P1, B2, P2)))),  # Infers B2 is at a safe pose wrt B1 at P1
    ]

    ####################

    cond_streams = []
    constants = []

    initial_atoms = [
        AtConf(initial_config),
        HandEmpty()
    ] + [
        AtPose(block, pose) for block, pose in initial_poses.iteritems()
    ] + [
        LegalKin(i, i) for i in range(num_poses)
    ] + [
        CollisionFree(b1, p1, b2, p2) for b1, p1, b2, p2 in product(blocks, range(num_poses), blocks, range(num_poses)) if p1 != p2 and b1 != b2
    ]

    goal_literals = [AtPose(block, pose)
                     for block, pose in goal_poses.iteritems()]

    problem = STRIPStreamProblem(
        initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

    return problem

##################################################

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.pddl.utils import convert_plan


def main():
    """
    Creates and solves the 1D task and motion planning STRIPStream problem.
    """

    problem = create_problem()
    print problem
    plan, _ = incremental_planner(problem)
    print
    print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
    main()
