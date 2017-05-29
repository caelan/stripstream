#!/usr/bin/env python

import argparse
import sys

from openravepy import RaveSetDebugLevel, DebugLevel, Environment, RaveDestroy, databases, interfaces
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.simple_focused import simple_focused
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan

from stripstream.pddl.examples.openrave_tamp.problems import dantam_distract

from manipulation.motion.single_query import vector_traj_helper
from stripstream.pddl.examples.openrave_tamp.openrave_tamp_utils import open_gripper, solve_inverse_kinematics, \
    set_manipulator_conf, Conf, Traj, initialize_openrave, \
    sample_manipulator_trajectory, manip_from_pose_grasp, execute_viewer, \
    manipulator_motion_plan, linear_motion_plan
from stripstream.pddl.examples.openrave_tamp.transforms import set_pose, object_trans_from_manip_trans, trans_from_point

from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import MultiEasyGenStream, EasyTestStream
from stripstream.utils import SEPARATOR, INF
import numpy as np

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

####################

PROBLEM = lambda env: dantam_distract(env, 8)
ARM = 'leftarm'
MAX_GRASPS = 1
DISABLE_MOTIONS = False
DISABLE_MOTION_COLLISIONS = False
APPROACH_VECTOR = 0.15 * np.array([0, 0, -1])

assert not DISABLE_MOTIONS or DISABLE_MOTION_COLLISIONS
if DISABLE_MOTIONS:
    print 'Warning: trajectories are disabled'
if DISABLE_MOTION_COLLISIONS:
    print 'Warning: trajectory collisions are disabled'

####################

# Types
OBJ, POSE, GRASP = Type(), Type(), Type()
CONF, TRAJ = Type(), Type()

####################

# Fluents
ConfEq = Pred(CONF)
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)
HandEmpty = Pred()

####################

# Derived
SafePose = Pred(OBJ, POSE)
SafeTraj = Pred(OBJ, TRAJ)

####################

# Static trajectory
FreeMotion = Pred(CONF, CONF, TRAJ)
HoldingMotion = Pred(CONF, CONF, GRASP, TRAJ)
GraspMotion = Pred(POSE, GRASP, CONF, TRAJ)

# Static collision
CFreePose = Pred(POSE, POSE)
CFreeTraj = Pred(TRAJ, POSE)

####################

O, P, G = Param(OBJ), Param(POSE), Param(GRASP)
O2, P2 = Param(OBJ), Param(POSE)
Q, Q2 = Param(CONF), Param(CONF)
T = Param(TRAJ)

rename_easy(locals())

####################

actions = [
    Action(name='pick', parameters=[O, P, G, Q, T],
           condition=And(PoseEq(O, P), HandEmpty(), ConfEq(Q), GraspMotion(P, G, Q, T),
                         ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
           # ForAll([O2], Or(Equal(O, O2), And(SafePose(O2, P), SafeGTraj(O2,
           # GT))))),
           effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),

    Action(name='place', parameters=[O, P, G, Q, T],
           condition=And(GraspEq(O, G), ConfEq(Q), GraspMotion(P, G, Q, T),
                         ForAll([O2], Or(Equal(O, O2), And(SafePose(O2, P), SafeTraj(O2, T))))),
           effect=And(PoseEq(O, P), HandEmpty(), Not(GraspEq(O, G)))),

    Action(name='move', parameters=[Q, Q2, T],
           condition=And(ConfEq(Q), HandEmpty(), FreeMotion(Q, Q2, T),
                         ForAll([O2], SafeTraj(O2, T))),
           effect=And(ConfEq(Q2), Not(ConfEq(Q)))),

    Action(name='move_holding', parameters=[Q, Q2, T, O, G],
           condition=And(ConfEq(Q), GraspEq(O, G), HoldingMotion(Q, Q2, G, T),
                         ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
           effect=And(ConfEq(Q2), Not(ConfEq(Q)))),
]

axioms = [
    Axiom(effect=SafePose(O2, P), condition=Exists(
        [P2], And(PoseEq(O2, P2), CFreePose(P, P2)))),
    Axiom(effect=SafeTraj(O2, T), condition=Exists(
        [P2], And(PoseEq(O2, P2), CFreeTraj(T, P2)))),
]

##################################################


def solve_tamp(env):
    viewer = env.GetViewer() is not None
    problem = PROBLEM(env)

    robot = env.GetRobots()[0]
    initialize_openrave(env, ARM, min_delta=.01)
    manipulator = robot.GetActiveManipulator()
    base_manip = interfaces.BaseManipulation(
        robot, plannername=None, maxvelmult=None)

    bodies = {obj: env.GetKinBody(obj) for obj in problem.object_names}
    all_bodies = bodies.values()
    # NOTE - assuming all objects has the same geometry
    assert len({body.GetKinematicsGeometryHash() for body in all_bodies}) == 1
    body1 = all_bodies[-1]  # Generic object 1
    body2 = all_bodies[-2] if len(bodies) >= 2 else body1  # Generic object 2
    grasps = problem.known_grasps[:MAX_GRASPS] if problem.known_grasps else []
    poses = problem.known_poses if problem.known_poses else []

    open_gripper(manipulator)
    initial_conf = Conf(robot.GetConfigurationValues()
                        [manipulator.GetArmIndices()])

    def _enable_all(enable):  # Enables or disables all bodies for collision checking
        for body in all_bodies:
            body.Enable(enable)

    ####################

    # Collision free test between an object at pose1 and an object at pose2
    def cfree_pose(pose1, pose2):
        body1.Enable(True)
        set_pose(body1, pose1.value)
        body2.Enable(True)
        set_pose(body2, pose2.value)
        return not env.CheckCollision(body1, body2)

    # Collision free test between a robot executing traj and an object at pose
    def _cfree_traj_pose(traj, pose):
        _enable_all(False)
        body2.Enable(True)
        set_pose(body2, pose.value)
        for conf in traj.value:
            set_manipulator_conf(manipulator, conf)
            if env.CheckCollision(robot, body2):
                return False
        return True

    # Collision free test between an object held at grasp while executing traj
    # and an object at pose
    def _cfree_traj_grasp_pose(traj, grasp, pose):
        _enable_all(False)
        body1.Enable(True)
        body2.Enable(True)
        set_pose(body2, pose.value)
        for conf in traj.value:
            set_manipulator_conf(manipulator, conf)
            manip_trans = manipulator.GetTransform()
            set_pose(body1, object_trans_from_manip_trans(
                manip_trans, grasp.value))
            if env.CheckCollision(body1, body2):
                return False
        return True

    def cfree_traj(traj, pose):  # Collision free test between a robot executing traj (which may or may not involve a grasp) and an object at pose
        if DISABLE_MOTION_COLLISIONS:
            return True
        return _cfree_traj_pose(traj, pose) and (traj.grasp is None or _cfree_traj_grasp_pose(traj, traj.grasp, pose))

    ####################

    # Sample pregrasp config and motion plan that performs a grasp
    def sample_grasp_traj(pose, grasp):
        _enable_all(False)
        body1.Enable(True)
        set_pose(body1, pose.value)
        manip_trans = manip_from_pose_grasp(pose.value, grasp.value)
        grasp_conf = solve_inverse_kinematics(
            manipulator, manip_trans)  # Grasp configuration
        if grasp_conf is None:
            return
        if DISABLE_MOTIONS:
            yield [(Conf(grasp_conf), Traj([]))]
            return

        set_manipulator_conf(manipulator, grasp_conf)
        robot.Grab(body1)

        pregrasp_trans = manip_trans.dot(trans_from_point(*APPROACH_VECTOR))
        pregrasp_conf = solve_inverse_kinematics(
            manipulator, pregrasp_trans)  # Pre-grasp configuration
        if pregrasp_conf is None:
            return
        path = linear_motion_plan(robot, pregrasp_conf)
        # grasp_traj = vector_traj_helper(env, robot, approach_vector) # Trajectory from grasp configuration to pregrasp
        #grasp_traj = workspace_traj_helper(base_manip, approach_vector)
        robot.Release(body1)
        if path is None:
            return
        grasp_traj = Traj(path)
        grasp_traj.grasp = grasp
        yield [(Conf(pregrasp_conf), grasp_traj)]

    def sample_free_motion(conf1, conf2):  # Sample motion while not holding
        if DISABLE_MOTIONS:
            #traj = Traj([conf1.value, conf2.value])
            traj = Traj([conf2.value])
            traj.grasp = None
            yield [(traj,)]
            return
        _enable_all(False)
        set_manipulator_conf(manipulator, conf1.value)
        #traj = cspace_traj_helper(base_manip, cspace, conf2.value, max_iterations=10)
        path = manipulator_motion_plan(
            base_manip, manipulator, conf2.value, max_iterations=10)
        if path is None:
            return
        traj = Traj(path)
        traj.grasp = None
        yield [(traj,)]

    def sample_holding_motion(conf1, conf2, grasp):  # Sample motion while holding
        if DISABLE_MOTIONS:
            #traj = Traj([conf1.value, conf2.value])
            traj = Traj([conf2.value])
            traj.grasp = grasp
            yield [(traj,)]
            return
        _enable_all(False)
        body1.Enable(True)
        set_manipulator_conf(manipulator, conf1.value)
        manip_trans = manipulator.GetTransform()
        set_pose(body1, object_trans_from_manip_trans(
            manip_trans, grasp.value))
        robot.Grab(body1)
        #traj = cspace_traj_helper(base_manip, cspace, conf2.value, max_iterations=10)
        path = manipulator_motion_plan(
            base_manip, manipulator, conf2.value, max_iterations=10)
        robot.Release(body1)
        if path is None:
            return
        traj = Traj(path)
        traj.grasp = grasp
        yield [(traj,)]

    ####################

    cond_streams = [
        # Pick/place trajectory
        MultiEasyGenStream(inputs=[P, G], outputs=[Q, T], conditions=[],
                           effects=[GraspMotion(P, G, Q, T)], generator=sample_grasp_traj),

        # Move trajectory
        MultiEasyGenStream(inputs=[Q, Q2], outputs=[T], conditions=[],
                           effects=[FreeMotion(Q, Q2, T)], generator=sample_free_motion, order=1, max_level=0),
        MultiEasyGenStream(inputs=[Q, Q2, G], outputs=[T], conditions=[],
                           effects=[HoldingMotion(Q, Q2, G, T)], generator=sample_holding_motion, order=1, max_level=0),

        # Collisions
        EasyTestStream(inputs=[P, P2], conditions=[], effects=[CFreePose(P, P2)],
                       test=cfree_pose, eager=True),
        EasyTestStream(inputs=[T, P], conditions=[], effects=[CFreeTraj(T, P)],
                       test=cfree_traj),
    ]

    ####################

    constants = map(GRASP, grasps) + map(POSE, poses)
    initial_atoms = [
        ConfEq(initial_conf),
        HandEmpty(),
    ] + [
        PoseEq(obj, pose) for obj, pose in problem.initial_poses.iteritems()
    ]
    goal_formula = And(ConfEq(initial_conf), *(PoseEq(obj, pose)
                                               for obj, pose in problem.goal_poses.iteritems()))
    stream_problem = STRIPStreamProblem(
        initial_atoms, goal_formula, actions + axioms, cond_streams, constants)

    if viewer:
        raw_input('Start?')
    search_fn = get_fast_downward('eager', max_time=10, verbose=False)
    #solve = lambda: incremental_planner(stream_problem, search=search_fn, frequency=1, waves=True, debug=False)
    solve = lambda: simple_focused(stream_problem, search=search_fn,
                                   max_level=INF, shared=False, debug=False, verbose=False, max_time=15)
    env.Lock()
    plan, universe = solve()
    env.Unlock()

    print SEPARATOR

    plan = convert_plan(plan)
    if plan is not None:
        print 'Success'
        for i, (action, args) in enumerate(plan):
            print i + 1, action, args
    else:
        print 'Failure'

    ####################

    def _execute_traj(confs):
        for j, conf in enumerate(confs):
            set_manipulator_conf(manipulator, conf)
            raw_input('%s/%s) Step?' % (j, len(confs)))

    if viewer and plan is not None:
        print SEPARATOR
        # Resets the initial state
        set_manipulator_conf(manipulator, initial_conf.value)
        for obj, pose in problem.initial_poses.iteritems():
            set_pose(bodies[obj], pose.value)

        for i, (action, args) in enumerate(plan):
            raw_input('\n%s/%s) Next?' % (i, len(plan)))
            if action.name == 'move':
                _, _, traj = args
                _execute_traj(traj.value)
            elif action.name == 'move_holding':
                _, _, traj, _, _ = args
                _execute_traj(traj.value)
            elif action.name == 'pick':
                obj, _, _, _, traj = args
                _execute_traj(traj.value[::-1])
                robot.Grab(bodies[obj])
                _execute_traj(traj.value)
            elif action.name == 'place':
                obj, _, _, _, traj = args
                _execute_traj(traj.value[::-1])
                robot.Release(bodies[obj])
                _execute_traj(traj.value)
            else:
                raise ValueError(action.name)
            env.UpdatePublishedBodies()
    raw_input('Finish?')

##################################################


def main(argv):
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    env = Environment()
    try:
        execute = lambda: solve_tamp(env)
        if args.viewer:
            execute_viewer(env, execute)
        else:
            execute()
    finally:
        if env.GetViewer() is not None:
            env.GetViewer().quitmainloop()
        RaveDestroy()
    print 'Done!'
    # TODO - it's segfaulting here...

if __name__ == '__main__':
    main(sys.argv[1:])
