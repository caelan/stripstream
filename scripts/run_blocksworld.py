#!/usr/bin/env python

from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.connectives import Not, And
from stripstream.pddl.operators import Action
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem

"""
(define (domain blocksworld)
  (:requirements :strips :equality)
  (:predicates (clear ?x)
               (on-table ?x)
               (arm-empty)
               (holding ?x)
               (on ?x ?y))

  (:action pickup
    :parameters (?ob)
    :precondition (and (clear ?ob) (on-table ?ob) (arm-empty))
    :effect (and (holding ?ob) (not (clear ?ob)) (not (on-table ?ob))
                 (not (arm-empty))))

  (:action putdown
    :parameters  (?ob)
    :precondition (and (holding ?ob))
    :effect (and (clear ?ob) (arm-empty) (on-table ?ob)
                 (not (holding ?ob))))

  (:action stack
    :parameters  (?ob ?underob)
    :precondition (and  (clear ?underob) (holding ?ob))
    :effect (and (arm-empty) (clear ?ob) (on ?ob ?underob)
                 (not (clear ?underob)) (not (holding ?ob))))

  (:action unstack
    :parameters  (?ob ?underob)
    :precondition (and (on ?ob ?underob) (clear ?ob) (arm-empty))
    :effect (and (holding ?ob) (clear ?underob)
                 (not (on ?ob ?underob)) (not (clear ?ob)) (not (arm-empty)))))

(define (problem pb2)
   (:domain blocksworld)
   (:objects a b)
   (:init (on-table a) (on-table b)  (clear a)  (clear b) (arm-empty))
   (:goal (and (on a b))))
"""


def create_problem():
    """
    Creates a blocksworld STRIPStream problem.

    :return: a :class:`.STRIPStreamProblem`
    """

    # Data types
    BLOCK = Type()

    # Fluent predicates
    Clear = Pred(BLOCK)
    OnTable = Pred(BLOCK)
    ArmEmpty = Pred()
    Holding = Pred(BLOCK)
    On = Pred(BLOCK, BLOCK)

    # Free parameters
    B1, B2 = Param(BLOCK), Param(BLOCK)

    rename_easy(locals())

    actions = [
        Action(name='pickup', parameters=[B1],
               condition=And(Clear(B1), OnTable(B1), ArmEmpty()),
               effect=And(Holding(B1), Not(Clear(B1)), Not(OnTable(B1)), Not(ArmEmpty()))),

        Action(name='putdown', parameters=[B1],
               condition=Holding(B1),
               effect=And(Clear(B1), OnTable(B1), ArmEmpty(), Not(Holding(B1)))),

        Action(name='stack', parameters=[B1, B2],
               condition=And(Clear(B2), Holding(B1)),
               effect=And(Clear(B1), On(B1, B2), ArmEmpty(), Not(Clear(B2)), Not(Holding(B1)))),

        Action(name='unstack', parameters=[B1, B2],
               condition=And(Clear(B1), On(B1, B2), ArmEmpty()),
               effect=And(Clear(B2), Holding(B1), Not(Clear(B1)), Not(On(B1, B2)), Not(ArmEmpty()))),
    ]

    axioms = []
    cond_streams = []
    constants = []

    initial_atoms = [
        OnTable('A'),
        OnTable('B'),
        OnTable('C'),
        Clear('A'),
        Clear('B'),
        Clear('C'),
        ArmEmpty(),
    ]

    goal_literals = [On('A', 'B'), On('B', 'C')]

    return STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

##################################################

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.pddl.utils import convert_plan


def main():
    """
    Creates and solves a blocksworld STRIPStream problem.
    """

    problem = create_problem()
    print problem
    plan, _ = incremental_planner(problem)
    print
    print 'Plan:', convert_plan(plan)

# TODO - visualize by applying actions to env state

if __name__ == '__main__':
    main()
