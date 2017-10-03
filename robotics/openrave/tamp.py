from itertools import islice

import numpy as np

from robotics.openrave.utils import solve_inverse_kinematics,  set_manipulator_conf, Conf, Traj, Grasp, Pose, manip_from_pose_grasp,  manipulator_motion_plan, base_motion_plan, linear_motion_plan, top_grasps, side_grasps, random_inverse_reachability,  mp_birrt, set_active
from robotics.openrave.transforms import set_pose,  object_trans_from_manip_trans, trans_from_point, set_trans, set_full_config, base_values_from_full_config,  full_config_from_base_values
from time import sleep


APPROACH_VECTOR = 0.15 * np.array([0, 0, -1])
DISABLE_MOTIONS = True
DISABLE_MOTION_COLLISIONS = True

assert not DISABLE_MOTIONS or DISABLE_MOTION_COLLISIONS
if DISABLE_MOTIONS:
    print 'Warning: trajectories are disabled'
if DISABLE_MOTION_COLLISIONS:
    print 'Warning: trajectory collisions are disabled'


def get_bodies(env):
    return {body.GetName(): body for body in env.GetBodies() if not body.IsRobot()}


def enable_all(bodies, enable):
    for body in bodies.values():
        body.Enable(enable)


def make_traj(path, obj=None, grasp=None, pose=None):
    traj = Traj(path)
    traj.obj = obj
    traj.grasp = grasp
    traj.pose = pose
    return traj


class Manipulation(object):
    pass


def get_stable_test(bodies, surfaces):
    def stable_test(obj, pose, surface):
        bodies[obj].Enable(True)
        set_pose(bodies[obj], pose.value)
        return surfaces[surface].supports(bodies[obj])
    return stable_test


def base_generator_fn(ir_model, max_failures=20):
    env = ir_model.env
    robot = ir_model.robot

    def base_generator(obj, pose, grasp):

        manip_tform = manip_from_pose_grasp(pose.value, grasp.value)
        generator = random_inverse_reachability(ir_model, manip_tform)
        while True:
            for _ in range(max_failures):
                base_tform = next(generator)
                if base_tform is None:
                    continue
                set_trans(robot, base_tform)
                if env.CheckCollision(robot) or robot.CheckSelfCollision():
                    continue
                yield base_tform
                break
            else:
                return
    return base_generator


def grasp_generator_fn(bodies, use_top_grasps, use_side_grasps, max_grasps):
    def grasp_generator(obj):
        grasps = []
        if use_top_grasps:
            grasps += list(top_grasps(bodies[obj]))
        if use_side_grasps:
            grasps += list(side_grasps(bodies[obj]))
        yield [(Grasp(grasp),) for grasp in grasps[:max_grasps]]
    return grasp_generator


def pose_generator_fn(bodies, surfaces):
    def pose_generator(obj, surface):
        while True:
            bodies[obj].Enable(True)
            pose = surfaces[surface].sample_placement(bodies[obj])
            if pose is None:
                break
            yield [(Pose(pose),)]
    return pose_generator


def cfree_pose_fn(env, bodies):

    def cfree_pose(obj1, pose1, obj2, pose2):
        body1 = bodies[obj1]
        body1.Enable(True)
        set_pose(body1, pose1.value)

        body2 = bodies[obj2]
        body2.Enable(True)
        set_pose(body2, pose2.value)
        return not env.CheckCollision(body1, body2)
    return cfree_pose


"""
def cfree_traj_fn(env, robot, manipulator, body1, body2, all_bodies):
  def _cfree_traj_pose(traj, pose): # Collision free test between a robot executing traj and an object at pose
    enable_all(all_bodies, False)
    body2.Enable(True)
    set_pose(body2, pose.value)
    for conf in traj.value:
      set_manipulator_conf(manipulator, conf)
      if env.CheckCollision(robot, body2):
        return False
    return True

  def _cfree_traj_grasp_pose(traj, grasp, pose): # Collision free test between an object held at grasp while executing traj and an object at pose
    enable_all(all_bodies, False)
    body1.Enable(True)
    body2.Enable(True)
    set_pose(body2, pose.value)
    for conf in traj.value:
      set_manipulator_conf(manipulator, conf)
      manip_trans = manipulator.GetTransform()
      set_pose(body1, object_trans_from_manip_trans(manip_trans, grasp.value))
      if env.CheckCollision(body1, body2):
        return False
    return True

  def cfree_traj(traj, pose): # Collision free test between a robot executing traj (which may or may not involve a grasp) and an object at pose
    if DISABLE_MOTION_COLLISIONS:
      return True
    if traj.pose is not None and traj.pose == pose:
      # This is the same pose of the manipulation
      return True
    return _cfree_traj_pose(traj, pose) and (traj.grasp is None or _cfree_traj_grasp_pose(traj, traj.grasp, pose))
  return cfree_traj
"""


def sample_grasp_traj_fn(ir_model, bodies, carry_config, max_failures=100):
    env = ir_model.env
    manipulator = ir_model.manip

    robot = ir_model.robot

    def sample_grasp_traj(obj, pose, grasp):
        enable_all(bodies, False)
        body = bodies[obj]
        body.Enable(True)
        set_pose(body, pose.value)

        set_manipulator_conf(manipulator, carry_config)
        manip_tform = manip_from_pose_grasp(pose.value, grasp.value)
        for base_tform in islice(random_inverse_reachability(ir_model, manip_tform), max_failures):
            set_trans(robot, base_tform)
            if env.CheckCollision(robot) or robot.CheckSelfCollision():
                continue
            approach_robot_conf = robot.GetConfigurationValues()

            grasp_manip_conf = solve_inverse_kinematics(
                manipulator, manip_tform)
            if grasp_manip_conf is None:
                continue
            set_manipulator_conf(manipulator, grasp_manip_conf)
            grasp_robot_conf = robot.GetConfigurationValues()
            if DISABLE_MOTIONS:
                path = [approach_robot_conf,
                        grasp_robot_conf, approach_robot_conf]
                yield [(Conf(approach_robot_conf), make_traj(path, obj, pose, grasp))]
                return

            robot.Grab(body)
            pregrasp_tform = manip_tform.dot(
                trans_from_point(*APPROACH_VECTOR))
            pregrasp_conf = solve_inverse_kinematics(
                manipulator, pregrasp_tform)
            if pregrasp_conf is None:
                continue

            path = linear_motion_plan(robot, pregrasp_conf)
            robot.Release(body)
            if path is None:
                continue

            yield [(Conf(approach_robot_conf), make_traj(path, obj, pose, grasp))]
            return
    return sample_grasp_traj


def sample_free_motion_fn(manipulator, base_manip, all_bodies):
    def sample_free_motion(conf1, conf2):
        if DISABLE_MOTIONS:
            yield [(make_traj([conf2.value]),)]
            return
        enable_all(all_bodies, False)
        set_manipulator_conf(manipulator, conf1.value)
        path = manipulator_motion_plan(
            base_manip, manipulator, conf2.value, max_iterations=10)
        if path is None:
            return
        yield [(make_traj(path),)]
    return sample_free_motion


def sample_free_base_motion_fn(base_manip, bodies):
    def sample_free_base_motion(conf1, conf2):

        enable_all(bodies, False)
        base_manip.robot.SetConfigurationValues(conf1.value)
        base_conf2 = base_values_from_full_config(conf2.value)

        base_path = base_motion_plan(
            base_manip, base_conf2, max_iterations=100)

        if base_path is None:
            return
        full_path = [full_config_from_base_values(
            base_conf, conf1.value) for base_conf in base_path]
        yield [(make_traj(full_path),)]
    return sample_free_base_motion


def sample_holding_motion_fn(robot, manipulator, base_manip, bodies):
    def sample_holding_motion(conf1, conf2, obj, grasp):
        if DISABLE_MOTIONS:
            yield [(make_traj([conf2.value], obj=obj, grasp=grasp),)]
            return
        enable_all(bodies, False)
        body = bodies[obj]
        body.Enable(True)
        set_manipulator_conf(manipulator, conf1.value)
        manip_trans = manipulator.GetTransform()
        set_pose(body, object_trans_from_manip_trans(manip_trans, grasp.value))
        robot.Grab(body)

        path = manipulator_motion_plan(
            base_manip, manipulator, conf2.value, max_iterations=10)
        robot.Release(body)
        if path is None:
            return
        yield [(make_traj(path, obj=obj, grasp=grasp),)]
    return sample_holding_motion


def visualize_solution(env, initial_conf, initial_poses, robot, bodies, plan):
    def _execute_traj(confs):
        for j, conf in enumerate(confs):
            set_full_config(robot, conf)

            sleep(0.02)

    set_full_config(robot, initial_conf.value)
    for obj, pose in initial_poses.iteritems():
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
