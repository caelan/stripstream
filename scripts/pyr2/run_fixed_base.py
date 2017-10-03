#!/usr/bin/env python2

from time import sleep

from PyR2.graphics3D import Window3D
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan
from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import EasyTestStream, EasyGenStream
from stripstream.utils import SEPARATOR

from robotics.pyr2.problems import dantam2
from robotics.pyr2.utils import is_window_active, inverse_kinematics, Traj, \
    manip_from_pose_grasp, approach_from_manip, get_name, get_wrist_frame, get_sample_fn, get_extend_fn, \
    get_distance_fn, get_collision_fn, draw_window_3d, TOP_GRASPS
from motion_planners.rrt_connect import birrt, direct_path

# TODO - extract out useful files and ignore the rest (kind of like what I
# did with FFRob)

####################

ARM = 'left'
MAX_GRASPS = 1
DISABLE_GRASP_TRAJECTORIES = False
DISABLE_MOTION_TRAJECTORIES = False
DISABLE_TRAJ_COLLISIONS = False
STEP_SIZE = 0.1

print SEPARATOR
if DISABLE_GRASP_TRAJECTORIES:
    print 'Warning: grasp trajectories are disabled'
if DISABLE_MOTION_TRAJECTORIES:
    print 'Warning: motion trajectories are disabled'
if DISABLE_TRAJ_COLLISIONS:
    print 'Warning: trajectory collisions are disabled'

####################

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

# Types
OBJ, POSE, GRASP = Type(), Type(), Type()
CONF, TRAJ = Type(), Type()

# Fluents
ConfEq = Pred(CONF)
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)
HandEmpty = Pred()

# Derived
SafePose = Pred(OBJ, POSE)
SafeTraj = Pred(OBJ, TRAJ)

# Static trajectory
FreeMotion = Pred(CONF, CONF, TRAJ)
HoldingMotion = Pred(CONF, CONF, GRASP, TRAJ)
GraspMotion = Pred(POSE, GRASP, CONF, TRAJ)

# Static collision
CFreePose = Pred(POSE, POSE)
CFreeTraj = Pred(TRAJ, POSE)

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
           effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),

    Action(name='place', parameters=[O, P, G, Q, T],
           condition=And(GraspEq(O, G), ConfEq(Q), GraspMotion(P, G, Q, T),
                         ForAll([O2], Or(Equal(O, O2), And(SafePose(O2, P), SafeTraj(O2, T))))),
           # ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
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

# TODO - self.robot.selfCollidePairs (in objects.pyx)


def amber_stripstream(use_window=True, step=False):
    problem = dantam2()
    window_3d = Window3D(title='World', windowWidth=500, noWindow=(not use_window),
                         viewport=problem.workspace.flatten('F'))

    if is_window_active(window_3d):
        draw_window_3d(window_3d, problem.initial_conf,
                       [body.applyTrans(pose) for body, pose in problem.initial_poses])
        raw_input('Start?')

    robot = problem.initial_conf.robot
    name_to_body = {get_name(body): body for body, _ in problem.initial_poses}
    static_bodies = [body.applyTrans(pose) for body, pose in problem.initial_poses
                     if get_name(body) not in problem.movable_names]
    block = name_to_body[problem.movable_names[-1]]  # Generic object

    ####################

    # NOTE - can either modify the world state or create new bodies
    # Collision free test between an object at pose1 and an object at pose2
    def cfree_pose(pose1, pose2):
        return not ((pose1 == pose2) or
                    block.applyTrans(pose1).collides(block.applyTrans(pose2)))

    def cfree_traj(traj, pose):  # Collision free test between a robot executing traj (which may or may not involve a grasp) and an object at pose
        if DISABLE_TRAJ_COLLISIONS:
            return True
        placed = [body.applyTrans(pose)]
        attached = None
        if traj.grasp is not None:
            attached = {ARM: body.applyTrans(
                get_wrist_frame(traj.grasp, robot, ARM).inverse())}
        # TODO - cache bodies for these configurations
        collision_fn = get_collision_fn(placed, attached)
        return not any(collision_fn(q) for q in traj.path)

    ####################

    # Sample pregrasp config and motion plan that performs a grasp
    def sample_grasp_traj(pose, grasp):
        attached = {ARM: body.applyTrans(
            get_wrist_frame(grasp, robot, ARM).inverse())}
        collision_fn = get_collision_fn(static_bodies, attached)

        manip_frame = manip_from_pose_grasp(pose, grasp, robot, ARM)
        grasp_conf = next(inverse_kinematics(
            problem.initial_conf, manip_frame, ARM), None)  # Grasp configuration
        if grasp_conf is None or collision_fn(grasp_conf):
            return

        # NOTE - I could also do this in the opposite order
        pregrasp_frame = approach_from_manip(manip_frame)
        pregrasp_conf = next(inverse_kinematics(
            grasp_conf, pregrasp_frame, ARM))  # Pregrasp configuration
        if pregrasp_conf is None or collision_fn(pregrasp_conf):
            return

        if DISABLE_GRASP_TRAJECTORIES:
            yield (pregrasp_conf, Traj([grasp_conf], grasp))
            return
        grasp_path = direct_path(grasp_conf, pregrasp_conf,  # Trajectory from grasp configuration to pregrasp
                                 get_extend_fn([robot.armChainNames[ARM]], STEP_SIZE), collision_fn)
        if grasp_path is None:
            return
        yield (pregrasp_conf, Traj(grasp_path, grasp))

    def sample_free_motion(conf1, conf2):  # Sample motion while not holding
        if DISABLE_MOTION_TRAJECTORIES:
            yield (Traj([conf2]),)
            return
        active_chains = [robot.armChainNames[ARM]]
        path = birrt(conf1, conf2, get_distance_fn(active_chains),
                     get_sample_fn(conf1, active_chains),
                     get_extend_fn(active_chains, STEP_SIZE), get_collision_fn(
                         static_bodies),
                     restarts=0, iterations=50, smooth=None)
        if path is None:
            return
        yield (Traj(path),)

    def sample_holding_motion(conf1, conf2, grasp):  # Sample motion while holding
        if DISABLE_MOTION_TRAJECTORIES:
            yield (Traj([conf2], grasp),)
            return
        active_chains = [robot.armChainNames[ARM]]
        attached = {ARM: body.applyTrans(
            get_wrist_frame(grasp, robot, ARM).inverse())}
        path = birrt(conf1, conf2, get_distance_fn(active_chains),
                     get_sample_fn(conf1, active_chains),
                     get_extend_fn(active_chains, STEP_SIZE), get_collision_fn(
                         static_bodies, attached),
                     restarts=0, iterations=50, smooth=None)
        if path is None:
            return
        yield (Traj(path, grasp),)

    ####################

    cond_streams = [
        # Pick/place trajectory
        EasyGenStream(inputs=[P, G], outputs=[Q, T], conditions=[],
                      effects=[GraspMotion(P, G, Q, T)], generator=sample_grasp_traj),

        # Move trajectory
        EasyGenStream(inputs=[Q, Q2], outputs=[T], conditions=[],
                      effects=[FreeMotion(Q, Q2, T)], generator=sample_free_motion, order=1, max_level=0),
        EasyGenStream(inputs=[Q, Q2, G], outputs=[T], conditions=[],
                      effects=[HoldingMotion(Q, Q2, G, T)], generator=sample_holding_motion, order=1, max_level=0),

        # Collisions
        EasyTestStream(inputs=[P, P2], conditions=[], effects=[CFreePose(P, P2)],
                       test=cfree_pose, eager=True),
        EasyTestStream(inputs=[T, P], conditions=[], effects=[CFreeTraj(T, P)],
                       test=cfree_traj),
    ]

    ####################

    constants = map(GRASP, TOP_GRASPS[:MAX_GRASPS]
                    ) + map(POSE, problem.known_poses)
    initial_atoms = [
        ConfEq(problem.initial_conf),
        HandEmpty(),
    ] + [
        PoseEq(get_name(body), pose) for body, pose in problem.initial_poses if get_name(body) in problem.movable_names
    ]
    goal_literals = [PoseEq(obj, pose)
                     for obj, pose in problem.goal_poses.iteritems()]
    if problem.goal_conf is not None:
        goal_literals.append(ConfEq(problem.goal_conf))
    stream_problem = STRIPStreamProblem(initial_atoms, And(
        *goal_literals), actions + axioms, cond_streams, constants)

    search_fn = get_fast_downward('eager', max_time=10, verbose=False)
    plan, _ = incremental_planner(
        stream_problem, search=search_fn, frequency=1, waves=True, debug=False)

    print SEPARATOR

    plan = convert_plan(plan)
    if plan is not None:
        print 'Success'
        for i, (action, args) in enumerate(plan):
            print i + 1, action
    else:
        print 'Failure'

    if is_window_active(window_3d) and plan is not None:
        robot_conf = problem.initial_conf
        placements = {get_name(body): body.applyTrans(pose)
                      for body, pose in problem.initial_poses}
        attached = {'left': None, 'right': None}

        def _execute_traj(path):
            for j, conf in enumerate(path):
                draw_window_3d(window_3d, conf, placements.values(), attached)
                if step:
                    raw_input('%s/%s) Step?' % (j, len(path)))
                else:
                    window_3d.update()
                    sleep(0.01)

        draw_window_3d(window_3d, robot_conf, placements.values())
        raw_input('Start?')
        for i, (action, args) in enumerate(plan):
            if action.name == 'move':
                _, _, traj = args
                _execute_traj(traj.path)
            elif action.name == 'move_holding':
                _, _, traj, _, _ = args
                _execute_traj(traj.path)
            elif action.name == 'pick':
                obj, _, grasp, _, traj = args
                _execute_traj(traj.path[::-1])
                attached[ARM] = name_to_body[obj].applyTrans(
                    get_wrist_frame(grasp, robot, ARM).inverse())
                del placements[obj]
                _execute_traj(traj.path)
            elif action.name == 'place':
                obj, pose, _, _, traj = args
                _execute_traj(traj.path[::-1])
                placements[obj] = name_to_body[obj].applyTrans(pose)
                attached[ARM] = None
                _execute_traj(traj.path)
            else:
                raise ValueError(action.name)

    raw_input('Finish?')

##################################################


def main():
    amber_stripstream()

if __name__ == '__main__':
    main()
