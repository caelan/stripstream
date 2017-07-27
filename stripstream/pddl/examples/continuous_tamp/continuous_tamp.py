from stripstream.pddl.logic.connectives import Not, Or, And
from stripstream.pddl.logic.quantifiers import Exists, ForAll
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.operators import Action, Axiom
from stripstream.utils import irange, INF
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyGenStream, EasyTestStream
from stripstream.pddl.objects import EasyType as Type, EasyParameter as Param
from stripstream.pddl.logic.predicates import EasyPredicate as Pred
from stripstream.pddl.examples.continuous_tamp.continuous_tamp_utils import are_colliding, in_region,  sample_region_pose, inverse_kinematics
from stripstream.pddl.utils import get_value

EAGER_TESTS = True

CONF, BLOCK, POSE, REGION = Type(), Type(), Type(), Type()

AtConf = Pred(CONF)
HandEmpty = Pred()
AtPose = Pred(BLOCK, POSE)
Holding = Pred(BLOCK)

Safe = Pred(BLOCK, BLOCK, POSE)
InRegion = Pred(BLOCK, REGION)


LegalKin = Pred(POSE, CONF)
CollisionFree = Pred(BLOCK, POSE, BLOCK, POSE)
Contained = Pred(BLOCK, POSE, REGION)
CanPlace = Pred(BLOCK, REGION)

IsSink = Pred(REGION)
IsStove = Pred(REGION)
Cleaned = Pred(BLOCK)
Cooked = Pred(BLOCK)

rename_easy(locals())


def compile_problem(tamp_problem):
    """
    Constructs a STRIPStream problem for the continuous TMP problem.

    :param tamp_problem: a :class:`.TMPProblem`
    :return: a :class:`.STRIPStreamProblem`
    """

    B1, B2 = Param(BLOCK), Param(BLOCK)
    P1, P2 = Param(POSE), Param(POSE)
    Q1, Q2 = Param(CONF), Param(CONF)
    R = Param(REGION)

    actions = [
        Action(name='pick', parameters=[B1, P1, Q1],
               condition=And(AtPose(B1, P1), HandEmpty(),
                             AtConf(Q1), LegalKin(P1, Q1)),
               effect=And(Holding(B1), Not(AtPose(B1, P1)), Not(HandEmpty()))),
        Action(name='place', parameters=[B1, P1, Q1],
               condition=And(Holding(B1), AtConf(Q1), LegalKin(P1, Q1),
                             ForAll([B2], Or(Equal(B1, B2), Safe(B2, B1, P1)))),
               effect=And(AtPose(B1, P1), HandEmpty(), Not(Holding(B1)))),
        Action(name='move', parameters=[Q1, Q2],
               condition=AtConf(Q1),
               effect=And(AtConf(Q2), Not(AtConf(Q1)))),

















        Action(name='clean', parameters=[B1, R],
               condition=And(InRegion(B1, R), IsSink(R)),
               effect=And(Cleaned(B1))),
        Action(name='cook', parameters=[B1, R],
               condition=And(InRegion(B1, R), IsStove(R)),
               effect=And(Cooked(B1))),
    ]

    axioms = [
        Axiom(effect=InRegion(B1, R), condition=Exists(
            [P1], And(AtPose(B1, P1), Contained(B1, P1, R)))),
        Axiom(effect=Safe(B2, B1, P1), condition=Exists(
            [P2], And(AtPose(B2, P2), CollisionFree(B1, P1, B2, P2)))),
    ]

    cond_streams = [
        EasyGenStream(inputs=[R, B1], outputs=[P1], conditions=[CanPlace(B1, R)], effects=[Contained(B1, P1, R)],
                      generator=lambda r, b: (sample_region_pose(r, b) for _ in irange(0, INF))),
        EasyGenStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[LegalKin(P1, Q1)],
                      generator=lambda p: iter([inverse_kinematics(p)])),
        EasyTestStream(inputs=[R, B1, P1], conditions=[], effects=[Contained(B1, P1, R)],
                       test=in_region, eager=EAGER_TESTS, plannable=False),
        EasyTestStream(inputs=[B1, P1, B2, P2], conditions=[], effects=[CollisionFree(B1, P1, B2, P2)],
                       test=lambda *args: not are_colliding(*args), eager=EAGER_TESTS),
    ]

    constants = [
        REGION(tamp_problem.env_region),
    ]

    initial_atoms = [
        AtConf(tamp_problem.initial_config),
    ] + [
        AtPose(block, pose) for block, pose in tamp_problem.initial_poses.iteritems()
    ] + [
        CanPlace(block, tamp_problem.env_region) for block in tamp_problem.initial_poses
    ]

    if tamp_problem.initial_holding is None:
        initial_atoms.append(HandEmpty())
    else:
        initial_atoms.append(Holding(tamp_problem.initial_holding))

    goal_literals = []
    if tamp_problem.goal_config is not None:
        goal_literals.append(AtConf(tamp_problem.goal_config))
    if tamp_problem.goal_holding is False:
        goal_literals.append(HandEmpty())
    elif tamp_problem.goal_holding is not None:
        goal_literals.append(Holding(tamp_problem.goal_holding))
    for block, goal in tamp_problem.goal_poses:
        goal_literals.append(AtPose(block, goal))
    for block, goal in tamp_problem.goal_regions:
        initial_atoms.append(CanPlace(block, goal))
        goal_literals.append(InRegion(block, goal))

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


def visualize_atoms(viewer, atoms):
    for atom in atoms:
        if atom.predicate == AtConf:
            q, = atom.args
            viewer.draw_robot(get_value(q))
        elif atom.predicate == AtPose:
            b, p = atom.args
            viewer.draw_block(get_value(b), get_value(p))
        elif atom.predicate == Holding:
            b, = atom.args
            viewer.draw_holding(get_value(b), get_config(atoms))


def visualize_initial(tamp_problem, planning_problem):
    from stripstream.pddl.examples.continuous_tamp.continuous_tamp_viewer import ContinuousTMPViewer
    viewer = ContinuousTMPViewer(
        tamp_problem.env_region, tamp_problem.get_regions(), title='Initial')
    visualize_atoms(viewer, planning_problem.initial_atoms)
    return viewer


def visualize_goal(tamp_problem, planning_problem):
    from stripstream.pddl.examples.continuous_tamp.continuous_tamp_viewer import ContinuousTMPViewer
    viewer = ContinuousTMPViewer(
        tamp_problem.env_region, tamp_problem.get_regions(), tl_y=300, title='Goal')
    visualize_atoms(viewer, planning_problem.goal_literals)
    return viewer
