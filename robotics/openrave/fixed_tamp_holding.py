from manipulation.motion.single_query import cspace_traj_helper, vector_traj_helper
from stripstream.pddl.examples.openrave.utils import solve_inverse_kinematics,  object_trans_from_manip_trans, set_manipulator_conf, Conf,  sample_manipulator_trajectory

from manipulation.bodies.robot import manip_from_pose_grasp
from manipulation.primitives.transforms import set_pose

DISABLE_TRAJECTORIES = True
DISABLE_TRAJ_COLLISIONS = True

assert not DISABLE_TRAJECTORIES or DISABLE_TRAJ_COLLISIONS
if DISABLE_TRAJECTORIES:
    print 'Warning: trajectories are disabled'
if DISABLE_TRAJ_COLLISIONS:
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


def cfree_traj_fn(env, robot, manipulator, body1, body2, all_bodies):
    def _cfree_traj_pose(traj, pose):
        enable_all(all_bodies, False)
        body2.Enable(True)
        set_pose(body2, pose.value)
        for conf in traj.path():
            set_manipulator_conf(manipulator, conf)
            if env.CheckCollision(robot, body2):
                return False
        return True

    def _cfree_traj_grasp_pose(traj, grasp, pose):
        enable_all(all_bodies, False)
        body1.Enable(True)
        body2.Enable(True)
        set_pose(body2, pose.value)
        for conf in traj.path():
            set_manipulator_conf(manipulator, conf)
            manip_trans = manipulator.GetTransform()
            set_pose(body1, object_trans_from_manip_trans(
                manip_trans, grasp.grasp_trans))
            if env.CheckCollision(body1, body2):
                return False
        return True

    def cfree_traj(traj, pose):
        if DISABLE_TRAJ_COLLISIONS:
            return True
        return _cfree_traj_pose(traj, pose) and (traj.grasp is None or
                                                 _cfree_traj_grasp_pose(traj, traj.grasp, pose))
    return cfree_traj


def sample_grasp_traj_fn(env, robot, manipulator, body1, all_bodies):
    def sample_grasp_traj(pose, grasp):
        enable_all(all_bodies, False)
        body1.Enable(True)
        set_pose(body1, pose.value)
        manip_trans, approach_vector = manip_from_pose_grasp(pose, grasp)
        grasp_conf = solve_inverse_kinematics(env, manipulator, manip_trans)
        if grasp_conf is None:
            return
        if DISABLE_TRAJECTORIES:
            yield [(Conf(grasp_conf), object())]
            return

        set_manipulator_conf(manipulator, grasp_conf)
        robot.Grab(body1)
        grasp_traj = vector_traj_helper(env, robot, approach_vector)

        robot.Release(body1)
        if grasp_traj is None:
            return
        grasp_traj.grasp = grasp
        pregrasp_conf = Conf(grasp_traj.end())
        yield [(pregrasp_conf, grasp_traj)]
    return sample_grasp_traj


def sample_free_motion_fn(manipulator, base_manip, cspace, all_bodies):
    def sample_free_motion(conf1, conf2):
        if DISABLE_TRAJECTORIES:
            yield [(object(),)]
            return
        enable_all(all_bodies, False)
        set_manipulator_conf(manipulator, conf1.value)

        traj = cspace_traj_helper(
            base_manip, cspace, conf2.value, max_iterations=10)
        if not traj:
            return
        traj.grasp = None
        yield [(traj,)]
    return sample_free_motion


def sample_holding_motion_fn(robot, manipulator, base_manip, cspace, body1, all_bodies):
    def sample_holding_motion(conf1, conf2, grasp):
        if DISABLE_TRAJECTORIES:
            yield [(object(),)]
            return
        enable_all(all_bodies, False)
        body1.Enable(True)
        set_manipulator_conf(manipulator, conf1.value)
        manip_trans = manipulator.GetTransform()
        set_pose(body1, object_trans_from_manip_trans(
            manip_trans, grasp.grasp_trans))
        robot.Grab(body1)

        traj = cspace_traj_helper(
            base_manip, cspace, conf2.value, max_iterations=10)
        robot.Release(body1)
        if not traj:
            return
        traj.grasp = grasp
        yield [(traj,)]
    return sample_holding_motion


def visualize_solution(env, problem, initial_conf, robot, manipulator, bodies, plan):
    def execute_traj(traj):

        path = list(sample_manipulator_trajectory(manipulator, traj.traj()))
        for j, conf in enumerate(path):
            set_manipulator_conf(manipulator, conf)
            raw_input('%s/%s) Step?' % (j, len(path)))

    set_manipulator_conf(manipulator, initial_conf.value)
    for obj, pose in problem.initial_poses.iteritems():
        set_pose(bodies[obj], pose.value)

    for i, (action, args) in enumerate(plan):
        raw_input('\n%s/%s) Next?' % (i, len(plan)))
        if action.name == 'move':
            _, _, traj = args
            execute_traj(traj)
        elif action.name == 'move_holding':
            _, _, traj, _, _ = args
            execute_traj(traj)
        elif action.name == 'pick':
            obj, _, _, _, traj = args
            execute_traj(traj.reverse())
            robot.Grab(bodies[obj])
            execute_traj(traj)
        elif action.name == 'place':
            obj, _, _, _, traj = args
            execute_traj(traj.reverse())
            robot.Release(bodies[obj])
            execute_traj(traj)
        else:
            raise ValueError(action.name)
        env.UpdatePublishedBodies()
