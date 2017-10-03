from time import sleep

import numpy as np

from robotics.openrave.utils import solve_inverse_kinematics,  set_manipulator_conf, Conf, Traj, manip_from_pose_grasp,  manipulator_motion_plan, linear_motion_plan,  has_mp, mp_birrt, mp_straight_line
from robotics.openrave.transforms import set_pose,  object_trans_from_manip_trans, trans_from_point


APPROACH_VECTOR = 0.15 * np.array([0, 0, -1])
DISABLE_MOTIONS = False
DISABLE_MOTION_COLLISIONS = False

assert not DISABLE_MOTIONS or DISABLE_MOTION_COLLISIONS
if DISABLE_MOTIONS:
    print 'Warning: trajectories are disabled'
if DISABLE_MOTION_COLLISIONS:
    print 'Warning: trajectory collisions are disabled'


def enable_all(all_bodies, enable):
    for body in all_bodies:
        body.Enable(enable)


def cfree_pose_fn(env, body1, body2):
    def cfree_pose(pose1, pose2):
        body1.Enable(True)
        set_pose(body1, pose1.value)
        body2.Enable(True)
        set_pose(body2, pose2.value)
        return not env.CheckCollision(body1, body2)
    return cfree_pose


def cfree_traj_fn(env, manipulator, body1, body2, all_bodies):
    robot = manipulator.GetRobot()

    def _cfree_traj_pose(traj, pose):
        enable_all(all_bodies, False)
        body2.Enable(True)
        set_pose(body2, pose.value)
        for conf in traj.value:
            set_manipulator_conf(manipulator, conf)
            if env.CheckCollision(robot, body2):
                return False
        return True

    def _cfree_traj_grasp_pose(traj, grasp, pose):
        enable_all(all_bodies, False)
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

    def cfree_traj(traj, pose):
        if DISABLE_MOTION_COLLISIONS:
            return True
        if traj.pose is not None and traj.pose == pose:

            return True
        return _cfree_traj_pose(traj, pose) and (traj.grasp is None or _cfree_traj_grasp_pose(traj, traj.grasp, pose))
    return cfree_traj


def sample_grasp_traj_fn(env, manipulator, body1, all_bodies):
    robot = manipulator.GetRobot()

    def sample_grasp_traj(pose, grasp):
        enable_all(all_bodies, False)
        body1.Enable(True)
        set_pose(body1, pose.value)
        manip_trans = manip_from_pose_grasp(pose.value, grasp.value)
        grasp_conf = solve_inverse_kinematics(manipulator, manip_trans)
        if grasp_conf is None:
            return
        if DISABLE_MOTIONS:
            yield [(Conf(grasp_conf), Traj([]))]
            return

        set_manipulator_conf(manipulator, grasp_conf)
        robot.Grab(body1)

        pregrasp_trans = manip_trans.dot(trans_from_point(*APPROACH_VECTOR))
        pregrasp_conf = solve_inverse_kinematics(manipulator, pregrasp_trans)
        if pregrasp_conf is None:
            return

        if has_mp():
            path = mp_straight_line(robot, grasp_conf, pregrasp_conf)
        else:
            path = linear_motion_plan(robot, pregrasp_conf)

        robot.Release(body1)
        if path is None:
            return
        grasp_traj = Traj(path)
        grasp_traj.pose = pose
        grasp_traj.grasp = grasp
        yield [(Conf(pregrasp_conf), grasp_traj)]
    return sample_grasp_traj


def sample_free_motion_fn(manipulator, base_manip, all_bodies):
    robot = manipulator.GetRobot()

    def sample_free_motion(conf1, conf2):
        if DISABLE_MOTIONS:

            traj = Traj([conf2.value])
            traj.pose = None
            traj.grasp = None
            yield [(traj,)]
            return
        enable_all(all_bodies, False)
        set_manipulator_conf(manipulator, conf1.value)

        if has_mp():
            path = mp_birrt(robot, conf1.value, conf2.value)
        else:

            path = manipulator_motion_plan(
                base_manip, manipulator, conf2.value, max_iterations=10)

        if path is None:
            return
        traj = Traj(path)
        traj.pose = None
        traj.grasp = None
        yield [(traj,)]
    return sample_free_motion


def sample_holding_motion_fn(manipulator, base_manip, body1, all_bodies):
    robot = manipulator.GetRobot()

    def sample_holding_motion(conf1, conf2, grasp):
        if DISABLE_MOTIONS:

            traj = Traj([conf2.value])
            traj.pose = None
            traj.grasp = grasp
            yield [(traj,)]
            return
        enable_all(all_bodies, False)
        body1.Enable(True)
        set_manipulator_conf(manipulator, conf1.value)
        manip_trans = manipulator.GetTransform()
        set_pose(body1, object_trans_from_manip_trans(
            manip_trans, grasp.value))
        robot.Grab(body1)

        if has_mp():
            path = mp_birrt(robot, conf1.value, conf2.value)
        else:

            path = manipulator_motion_plan(
                base_manip, manipulator, conf2.value, max_iterations=10)

        robot.Release(body1)
        if path is None:
            return
        traj = Traj(path)
        traj.pose = None
        traj.grasp = grasp
        yield [(traj,)]
    return sample_holding_motion


def visualize_solution(env, problem, initial_conf, robot, manipulator, bodies, plan):
    def _execute_traj(confs):
        for j, conf in enumerate(confs):
            set_manipulator_conf(manipulator, conf)
            sleep(0.05)

    set_manipulator_conf(manipulator, initial_conf.value)
    for obj, pose in problem.initial_poses.iteritems():
        set_pose(bodies[obj], pose.value)

    raw_input('Start?')
    for i, (action, args) in enumerate(plan):

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
