from stripstream.pddl.examples.countable_tamp.countable_tamp_utils import NUM_POSES
from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal, Atom
from stripstream.pddl.operators import Action, Axiom
from stripstream.utils import irange
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.utils import get_value
from stripstream.pddl.cond_streams import EasyGenStream, EasyTestStream
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.predicates import EasyPredicate as Pred

EAGER_TESTS = True
ROBOT_ROW = -1


CONF, BLOCK, POSE = Type(), Type(), Type()

AtConf = Pred(CONF)
AtPose = Pred(BLOCK, POSE)
HandEmpty = Pred()
Holding = Pred(BLOCK)


Safe = Pred(BLOCK, POSE)

LegalKin = Pred(POSE, CONF)

CollisionFree = Pred(POSE, POSE)

rename_easy(locals())


def compile_problem(tamp_problem):
    """
    Constructs a STRIPStream problem for the countable TMP problem.

    :param tamp_problem: a :class:`.TMPProblem`
    :return: a :class:`.STRIPStreamProblem`
    """

    B1, B2 = Param(BLOCK), Param(BLOCK)
    P1, P2 = Param(POSE), Param(POSE)
    Q1, Q2 = Param(CONF), Param(CONF)

    actions = [
        Action(name='pick', parameters=[B1, P1, Q1],
               condition=And(AtPose(B1, P1), HandEmpty(),
                             AtConf(Q1), LegalKin(P1, Q1)),
               effect=And(Holding(B1), Not(AtPose(B1, P1)), Not(HandEmpty()))),
        Action(name='place', parameters=[B1, P1, Q1],
               condition=And(Holding(B1), AtConf(Q1), LegalKin(P1, Q1),

                             ForAll([B2], Or(Equal(B1, B2), Safe(B2, P1)))),

               effect=And(AtPose(B1, P1), HandEmpty(), Not(Holding(B1)))),
        Action(name='move', parameters=[Q1, Q2],
               condition=AtConf(Q1),
               effect=And(AtConf(Q2), Not(AtConf(Q1)))),
    ]

    axioms = [

        Axiom(effect=Safe(B2, P1), condition=Exists(
            [P2], And(AtPose(B2, P2), CollisionFree(P1, P2)))),
    ]

    cond_streams = [



        EasyGenStream(inputs=[], outputs=[P1], conditions=[], effects=[],
                      generator=lambda: irange(0, NUM_POSES)),
        EasyGenStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[LegalKin(P1, Q1)],
                      generator=lambda p: iter([p])),


        EasyTestStream(inputs=[P1, P2], conditions=[], effects=[CollisionFree(P1, P2)],
                       test=lambda p1, p2: p1 != p2, eager=EAGER_TESTS),
    ]

    constants = []

    initial_atoms = [
        AtConf(tamp_problem.initial_config),
    ] + [
        AtPose(block, pose) for block, pose in tamp_problem.initial_poses.iteritems()
    ]
    if tamp_problem.initial_holding is False:
        initial_atoms.append(HandEmpty())
    else:
        initial_atoms.append(Holding(tamp_problem.initial_holding, BLOCK))

    goal_literals = []
    if tamp_problem.goal_holding is False:
        goal_literals.append(HandEmpty())
    elif tamp_problem.goal_holding is not None:
        goal_literals.append(Holding(tamp_problem.goal_holding))
    for block, goal in tamp_problem.goal_poses.iteritems():
        goal_literals.append(AtPose(block, goal))

    return STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)


COLORS = ['red', 'orange', 'yellow', 'green', 'blue', 'violet']


def get_config(atoms):
    for atom in atoms:
        if atom.predicate == AtConf:
            q, = atom.args
            return get_value(q)
    return None


def get_holding(atoms):
    for atom in atoms:
        if atom.predicate == Holding:
            b, = atom.args
            return get_value(b)
    return None


def get_color(b):
    return COLORS[int(b[len('block'):])]


def visualize_atoms(viewer, atoms):
    conf = get_config(atoms)
    holding = get_holding(atoms)
    viewer.draw_robot(ROBOT_ROW, conf)
    if holding is not None:
        viewer.draw_block(ROBOT_ROW, conf, color=get_color(holding))
    for atom in atoms:
        if isinstance(atom, Atom) and atom.predicate == AtPose:
            b, p = atom.args
            viewer.draw_block(0, get_value(p), color=get_color(get_value(b)))
