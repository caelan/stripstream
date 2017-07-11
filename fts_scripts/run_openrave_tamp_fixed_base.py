#!/usr/bin/env python2

import argparse
import sys

from openravepy import RaveSetDebugLevel, DebugLevel, Environment, RaveDestroy, databases, interfaces

from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.focused.simple_focused import simple_focused
from stripstream.algorithms.search.fast_downward import get_fast_downward

from stripstream.pddl.examples.openrave_tamp.openrave_tamp_utils import open_gripper, \
    Conf, initialize_openrave, execute_viewer

from stripstream.fts.constraint import Eq, ConType, Unconstrained
from stripstream.fts.variable import VarType, Par, Var, X, U, nX
from stripstream.fts.clause import Clause
from stripstream.fts.sampler import Sampler
from stripstream.fts.problem import FTSProblem
from stripstream.fts.stripstream_conversion import constraint_to_stripstream
from stripstream.fts.utils import convert_plan, rename_variables
from stripstream.utils import SEPARATOR, INF

from stripstream.pddl.examples.openrave_tamp.problems import dantam_distract
from robotics.openrave.tamp_fixed_base import cfree_pose_fn, cfree_traj_fn, \
    sample_grasp_traj_fn, sample_free_motion_fn, sample_holding_motion_fn, visualize_solution

####################

PROBLEM = lambda env: dantam_distract(env, 8)
ARM = 'leftarm'
MAX_GRASPS = 1

####################


def get_problem(env, problem, robot, manipulator, base_manip, bodies, initial_conf):
    all_bodies = bodies.values()
    # Assuming all objects has the same geometry
    assert len({body.GetKinematicsGeometryHash() for body in all_bodies}) == 1
    body1 = all_bodies[-1]  # Generic object 1
    body2 = all_bodies[-2] if len(bodies) >= 2 else body1  # Generic object 2
    grasps = problem.known_grasps[:MAX_GRASPS] if problem.known_grasps else []
    poses = problem.known_poses if problem.known_poses else []

    cfree_pose = cfree_pose_fn(env, body1, body2)
    cfree_traj = cfree_traj_fn(
        env, robot, manipulator, body1, body2, all_bodies)

    # Variables
    R_Q, O_P, O_H, R_T, = 'R_Q', 'O_P', 'O_H', 'R_T'

    # Types
    OBJ = VarType(domain=list(bodies))
    CONF, TRAJ = VarType(), VarType()
    BOOL = VarType(domain=[True, False])
    POSE = VarType(domain=poses + grasps)

    # Trajectory constraints
    FreeMotion = ConType([CONF, CONF, TRAJ])
    HoldingMotion = ConType([CONF, CONF, POSE, TRAJ])
    GraspMotion = ConType([POSE, POSE, CONF, TRAJ])

    Stable = ConType([POSE], satisfying=[(p,) for p in poses])
    Grasp = ConType([POSE], satisfying=[(g,) for g in grasps])

    # Static constraints
    CFreePose = ConType([POSE, POSE], test=cfree_pose)
    CFreeTraj = ConType([TRAJ, POSE], test=cfree_traj)

    O = Par(OBJ)
    P, G = Par(POSE), Par(POSE)
    O2, P2 = Par(OBJ), Par(POSE)
    Q, Q2 = Par(CONF), Par(CONF)
    T = Par(TRAJ)

    rename_variables(locals())

    state_vars = [
        Var(R_Q, CONF),
        Var(O_P, POSE, args=[OBJ]),
        Var(O_H, BOOL, args=[OBJ]),
    ]
    control_vars = [Var(R_T, TRAJ)]

    ####################

    transition = [
        Clause([Stable(X[O_P, O]), Grasp(nX[O_P, O]),
                Eq(X[O_H, O], False), Eq(nX[O_H, O], True),
                GraspMotion(X[O_P, O], nX[O_P, O], X[R_Q], U[R_T])] +
               [Eq(X[O_H, obj], False) for obj in bodies] +
               [CFreeTraj(U[R_T], X[O_P, obj]) for obj in bodies],
               name='pick'),

        Clause([Grasp(X[O_P, O]), Stable(nX[O_P, O]),
                Eq(X[O_H, O], True), Eq(nX[O_H, O], False),
                GraspMotion(nX[O_P, O], X[O_P, O], X[R_Q], U[R_T])] +
               [CFreePose(nX[O_P, O], X[O_P, obj]) for obj in bodies] +
               [CFreeTraj(U[R_T], X[O_P, obj]) for obj in bodies], name='place'),

        Clause([FreeMotion(X[R_Q], nX[R_Q], U[R_T])] +
               [Eq(X[O_H, obj], False) for obj in bodies] +
               [CFreeTraj(U[R_T], X[O_P, obj]) for obj in bodies],
               name='move'),

        Clause([HoldingMotion(X[R_Q], nX[R_Q], X[O_P, O], U[R_T]), Eq(X[O_H, O], True)] +
               [CFreeTraj(U[R_T], X[O_P, obj]) for obj in bodies],
               name='move_holding')
    ]

    ####################

    sample_grasp_traj = sample_grasp_traj_fn(
        env, robot, manipulator, body1, all_bodies)
    sample_free_motion = sample_free_motion_fn(
        manipulator, base_manip, all_bodies)
    sample_holding_motion = sample_holding_motion_fn(
        robot, manipulator, base_manip, body1, all_bodies)

    samplers = [
        # Pick/place trajectory
        Sampler([GraspMotion(P, G, Q, T)], gen=sample_grasp_traj,
                inputs=[P, G], domain=[Stable(P), Grasp(G)]),

        # Move trajectory
        Sampler([FreeMotion(Q, Q2, T)], gen=sample_free_motion,
                inputs=[Q, Q2], order=1, max_level=0),
        Sampler([HoldingMotion(Q, Q2, G, T)], gen=sample_holding_motion,
                inputs=[Q, Q2, G], domain=[Grasp(G)], order=1, max_level=0),
    ]

    ####################

    initial_state = [
        Eq(X[R_Q], initial_conf),
    ] + [
        Eq(X[O_P, obj], pose) for obj, pose in problem.initial_poses.iteritems()
    ] + [
        Eq(X[O_H, obj], False) for obj in problem.initial_poses
    ]

    goal_constraints = [Eq(X[R_Q], initial_conf)] + [
        Eq(X[O_P, obj], pose) for obj, pose in problem.goal_poses.iteritems()
    ]

    return FTSProblem(state_vars, control_vars, transition, samplers, initial_state, goal_constraints)

##################################################


def solve_tamp(env):
    viewer = env.GetViewer() is not None
    problem = PROBLEM(env)

    robot, manipulator, base_manip, _ = initialize_openrave(
        env, ARM, min_delta=.01)
    bodies = {obj: env.GetKinBody(obj) for obj in problem.object_names}

    open_gripper(manipulator)
    initial_conf = Conf(robot.GetConfigurationValues()
                        [manipulator.GetArmIndices()])

    ####################

    fts_problem = get_problem(
        env, problem, robot, manipulator, base_manip, bodies, initial_conf)
    print
    print fts_problem

    stream_problem = constraint_to_stripstream(fts_problem)
    print
    print stream_problem

    if viewer:
        raw_input('Start?')
    search_fn = get_fast_downward('eager', max_time=10)
    #solve = lambda: incremental_planner(stream_problem, search=search_fn, frequency=1, waves=True, debug=False)
    solve = lambda: simple_focused(
        stream_problem, search=search_fn, max_level=INF, shared=False)
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

    if viewer and plan is not None:
        print SEPARATOR
        visualize_solution(env, problem, initial_conf,
                           robot, manipulator, bodies, plan)
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

if __name__ == '__main__':
    main(sys.argv[1:])
