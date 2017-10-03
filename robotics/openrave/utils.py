from itertools import count
from random import uniform
import math
import sys

from openravepy import rotationMatrixFromAxisAngle, Sensor, RaveCreateCollisionChecker, databases, interfaces,  IkParameterization, GeometryType, RaveCreateKinBody, planning_error, openravepy_int,  CollisionOptionsStateSaver, DOFAffine, RaveGetEnvironment, RaveGetAffineDOFValuesFromTransform
import numpy as np

from openravepy.misc import SetViewerUserThread
from robotics.openrave.transforms import quat_from_axis_angle, trans_from_pose, manip_trans_from_object_trans,  length, normalize, trans_from_point, get_point,  trans_from_quat, trans_from_axis_angle, get_active_config, set_active_config, set_quat,  pose_from_quat_point
from stripstream.utils import irange, INF

MIN_DELTA = 0.01
SURFACE_Z_OFFSET = 1e-3

AFFINE_MASK = DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis
ROTATION_AXIS = (0, 0, 1)


class Wrapper(object):

    def __init__(self, value):
        self.id = next(self._ids)
        self.value = value

    def __repr__(self):
        return self.__class__.__name__ + '(%d)' % self.id


class Conf(Wrapper):
    _ids = count(0)


class Pose(Wrapper):
    _ids = count(0)


class Grasp(Wrapper):
    _ids = count(0)


class Traj(Wrapper):
    _ids = count(0)


def get_env(body):
    return RaveGetEnvironment(body.GetEnvironmentId())


def get_name(body):
    return str(body.GetName())


def get_geometries(body):
    return (geometry for link in body.GetLinks() for geometry in link.GetGeometries())


def in_env(body):
    return body.GetEnvironmentId != 0


def set_color(body, color):
    for geometry in get_geometries(body):
        geometry.SetDiffuseColor(color)


def set_transparency(body, transparency):
    for geometry in get_geometries(body):
        geometry.SetTransparency(transparency)


def get_box_dimensions(box):
    [link] = box.GetLinks()
    [geometry] = link.GetGeometries()
    assert geometry.GetType() == GeometryType.Box
    return geometry.GetBoxExtents()


def box_body(env, name, dx, dy, dz, color=None, transparency=None):
    body = RaveCreateKinBody(env, '')
    body.InitFromBoxes(
        np.array([[0, 0, .5 * dz, .5 * dx, .5 * dy, .5 * dz]]), draw=True)
    body.SetName(name)
    if color is not None:
        set_color(body, color)
    if transparency is not None:
        set_transparency(body, transparency)
    return body


def mirror_arm_config(robot, config):
    return config * np.array([1 if left_min == right_min else -1 for left_min, right_min in
                              zip(robot.GetDOFLimits(robot.GetManipulator('leftarm').GetArmIndices())[0],
                                  robot.GetDOFLimits(robot.GetManipulator('rightarm').GetArmIndices())[0])])


def set_manipulator_conf(manipulator, values):
    manipulator.GetRobot().SetDOFValues(values, manipulator.GetArmIndices())


def set_base_conf(body, base_values):
    trans = body.GetTransform()
    trans[:3, :3] = rotationMatrixFromAxisAngle(
        np.array([0, 0, base_values[-1]]))
    trans[:2, 3] = base_values[:2]
    body.SetTransform(trans)


def set_gripper(manipulator, values):
    manipulator.GetRobot().SetDOFValues(values, manipulator.GetGripperIndices())


def open_gripper(manipulator):
    _, upper = manipulator.GetRobot().GetDOFLimits(manipulator.GetGripperIndices())
    set_gripper(manipulator, upper)


def close_gripper(manipulator):
    lower, _ = manipulator.GetRobot().GetDOFLimits(manipulator.GetGripperIndices())
    set_gripper(manipulator, lower)


def random_inverse_reachability(ir_model, manip_trans):
    index_manip_iterator = [(manip_trans, manip_trans)]
    try:
        for base_trans, _, _ in ir_model.randomBaseDistributionIterator(index_manip_iterator):

            yield base_trans
    except (ValueError, planning_error):
        raise StopIteration


def openrave_inverse_reachability(ir_model, manip_trans):
    index_manip_iterator = [(manip_trans, manip_trans)]
    try:
        for base_trans, _, _ in ir_model.sampleBaseDistributionIterator(index_manip_iterator, logllthresh=-INF, Nprematuresamples=1):

            yield base_trans
    except (ValueError, planning_error):
        raise StopIteration


def solve_inverse_kinematics(manipulator, manip_trans):
    robot = manipulator.GetRobot()
    env = robot.GetEnv()
    with robot:
        robot.SetActiveDOFs(manipulator.GetArmIndices())

        config = manipulator.FindIKSolution(manip_trans, 0)
        if config is None:
            return None
        robot.SetDOFValues(config, manipulator.GetArmIndices())
        if env.CheckCollision(robot) or robot.CheckSelfCollision():
            return None
        return config


def get_collision_fn(env, body, self_collisions=False):
    def fn(q):
        body.SetActiveDOFValues(q)
        return env.CheckCollision(body) or (self_collisions and body.CheckSelfCollision())
    return fn


def get_sample_fn(env, body, collisions=False, self_collisions=False):
    limits = body.GetActiveDOFLimits()
    collision = get_collision_fn(env, body, self_collisions=self_collisions)

    def fn():
        while True:
            config = np.random.uniform(*limits)
            if not collisions or not collision(config):
                return config
    return fn


def get_distance_fn(body):
    weights = body.GetActiveDOFWeights()

    def distance_fn(q1, q2):
        diff = body.SubtractActiveDOFValues(q2, q1)
        return math.sqrt(np.dot(weights, diff * diff))
    return distance_fn


def get_extend_fn(body):
    resolutions = body.GetActiveDOFResolutions()

    def extend_fn(q1, q2):
        n = int(np.max(
            np.abs(np.divide(body.SubtractActiveDOFValues(q2, q1), resolutions)))) + 1
        q = q1
        for i in range(n):
            q = (1. / (n - i)) * body.SubtractActiveDOFValues(q2, q) + q
            yield q
    return extend_fn


def manip_from_pose_grasp(pose, grasp):
    return manip_trans_from_object_trans(trans_from_pose(pose), grasp)


def top_grasps(box):
    (w, l, h) = get_box_dimensions(box)
    origin = trans_from_point(0, 0, -h)
    bottom = trans_from_point(0, 0, -h)
    reflect = trans_from_quat(quat_from_axis_angle(0, -math.pi, 0))
    for i in range(4):
        rotate_z = trans_from_axis_angle(0, 0, i * math.pi / 2)
        yield reflect.dot(origin).dot(bottom).dot(rotate_z)


def side_grasps(box, under=True):
    (w, l, h) = get_box_dimensions(box)
    origin = trans_from_point(0, 0, -2 * h)
    for j in range(1 + under):
        swap_xz = trans_from_axis_angle(0, -math.pi / 2 + j * math.pi, 0)
        for i in range(4):
            rotate_z = trans_from_axis_angle(0, 0, i * math.pi / 2)
            yield swap_xz.dot(rotate_z).dot(origin)


def linear_interpolation(body, q1, q2):
    dq = body.SubtractActiveDOFValues(q2, q1)
    steps = np.abs(np.divide(dq, body.GetActiveDOFResolutions())) + 1
    n = int(np.max(steps))
    for i in range(n):
        yield q1 + (1. + i) / n * dq


def extract_config(manipulator, spec, data):
    return spec.ExtractJointValues(data, manipulator.GetRobot(), manipulator.GetArmIndices())


def sample_manipulator_trajectory(manipulator, traj):
    spec = traj.GetConfigurationSpecification()
    waypoints = [extract_config(manipulator, spec, traj.GetWaypoint(i))
                 for i in range(traj.GetNumWaypoints())]
    yield waypoints[0]
    for start, end in zip(waypoints, waypoints[1:]):
        for conf in linear_interpolation(manipulator.GetRobot(), start, end):
            yield conf


def extract_base_values(robot, spec, data):
    return RaveGetAffineDOFValuesFromTransform(spec.ExtractTransform(np.identity(4), data, robot), AFFINE_MASK, ROTATION_AXIS)


def sample_base_trajectory(robot, traj):
    spec = traj.GetConfigurationSpecification()
    waypoints = [extract_base_values(robot, spec, traj.GetWaypoint(
        i)) for i in range(traj.GetNumWaypoints())]
    yield waypoints[0]
    for start, end in zip(waypoints, waypoints[1:]):
        for conf in linear_interpolation(robot, start, end):
            yield conf


def execute_viewer(env, execute):
    if sys.platform == 'darwin':
        SetViewerUserThread(env, 'qtcoin', execute)
    else:
        env.SetViewer('qtcoin')
        execute()


def initialize_openrave(env, manipulator_name, min_delta=MIN_DELTA, collision_checker='ode'):
    env.StopSimulation()
    for sensor in env.GetSensors():
        sensor.Configure(Sensor.ConfigureCommand.PowerOff)
        sensor.Configure(Sensor.ConfigureCommand.RenderDataOff)
        sensor.Configure(Sensor.ConfigureCommand.RenderGeometryOff)
        env.Remove(sensor)
    env.SetCollisionChecker(RaveCreateCollisionChecker(env, collision_checker))
    env.GetCollisionChecker().SetCollisionOptions(0)

    assert len(env.GetRobots()) == 1
    robot = env.GetRobots()[0]

    cd_model = databases.convexdecomposition.ConvexDecompositionModel(robot)
    if not cd_model.load():
        print 'Generating convex decomposition model'
        cd_model.autogenerate()
    l_model = databases.linkstatistics.LinkStatisticsModel(robot)

    if not l_model.load():
        print 'Generating link statistics model'
        l_model.autogenerate()
    l_model.setRobotWeights()
    l_model.setRobotResolutions(xyzdelta=min_delta)

    robot.SetActiveManipulator(manipulator_name)
    manipulator = robot.GetManipulator(manipulator_name)
    robot.SetActiveDOFs(manipulator.GetArmIndices())
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot, iktype=IkParameterization.Type.Transform6D,
                                                                 forceikfast=True, freeindices=None, freejoints=None, manip=None)
    if not ikmodel.load():
        print 'Generating inverse kinematics model'
        ikmodel.autogenerate()

    base_manip = interfaces.BaseManipulation(
        robot, plannername=None, maxvelmult=None)
    ir_model = databases.inversereachability.InverseReachabilityModel(robot)
    if ir_model.load():
        print 'Generating inverse reachability model'
        ir_model.autogenerate()

    return robot, manipulator, base_manip, ir_model


def collision_saver(env, options):
    return CollisionOptionsStateSaver(env.GetCollisionChecker(), options)


def has_mp():
    try:
        import motion_planners
    except ImportError:
        return False
    return True


def mp_birrt(robot, q1, q2):
    from motion_planners.rrt_connect import birrt
    return birrt(q1, q2, get_distance_fn(robot),
                 get_sample_fn(robot.GetEnv(), robot),
                 get_extend_fn(robot), get_collision_fn(robot.GetEnv(), robot))


def mp_straight_line(robot, q1, q2):
    from motion_planners.rrt_connect import direct_path
    return direct_path(q1, q2, get_extend_fn(robot),
                       get_collision_fn(robot.GetEnv(), robot))


def linear_motion_plan(robot, end_config):
    env = robot.GetEnv()
    with robot:

        with collision_saver(env, openravepy_int.CollisionOptions.ActiveDOFs):
            start_config = get_active_config(robot)
            path = [start_config] + \
                list(linear_interpolation(robot, start_config, end_config))
            for conf in path:
                set_active_config(robot, conf)
                if env.CheckCollision(robot):
                    return None
            return path


def set_active(robot, indices=(), use_base=False):
    if use_base is False:
        robot.SetActiveDOFs(indices)
    robot.SetActiveDOFs(indices, AFFINE_MASK, ROTATION_AXIS)


def cspace_motion_plan(base_manip, indices, goal, step_length=MIN_DELTA, max_iterations=10, max_tries=1):
    with base_manip.robot:
        base_manip.robot.SetActiveDOFs(indices)
        with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
            try:
                traj = base_manip.MoveActiveJoints(goal=goal, steplength=step_length, maxiter=max_iterations, maxtries=max_tries,
                                                   execute=False, outputtraj=None, goals=None, outputtrajobj=True, jitter=None, releasegil=False, postprocessingplanner=None,
                                                   postprocessingparameters=None)

                return traj

            except planning_error:
                return None


def base_motion_plan(base_manip, goal, step_length=MIN_DELTA, max_iterations=10, max_tries=1):

    with base_manip.robot:
        set_active(base_manip.robot, use_base=True)
        with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
            try:
                traj = base_manip.MoveActiveJoints(goal=goal, steplength=step_length, maxiter=max_iterations, maxtries=max_tries,
                                                   execute=False, outputtraj=None, goals=None, outputtrajobj=True, jitter=None, releasegil=False, postprocessingplanner=None,
                                                   postprocessingparameters=None)

                return list(sample_base_trajectory(base_manip.robot, traj))
            except planning_error:
                return None


def manipulator_motion_plan(base_manip, manipulator, goal, step_length=MIN_DELTA, max_iterations=10, max_tries=1):
    with base_manip.robot:
        base_manip.robot.SetActiveManipulator(manipulator)
        base_manip.robot.SetActiveDOFs(manipulator.GetArmIndices())
        with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
            try:
                traj = base_manip.MoveManipulator(goal=goal,
                                                  maxiter=max_iterations, execute=False, outputtraj=None, maxtries=max_tries,
                                                  goals=None, steplength=step_length, outputtrajobj=True, jitter=None, releasegil=False)
                return list(sample_manipulator_trajectory(manipulator, traj))
            except planning_error:
                return None


def workspace_motion_plan(base_manip, manipulator, vector, steps=10):
    distance, direction = length(vector), normalize(vector)
    step_length = distance / steps
    with base_manip.robot:
        base_manip.robot.SetActiveManipulator(manipulator)
        base_manip.robot.SetActiveDOFs(manipulator.GetArmIndices())
        with collision_saver(base_manip.robot.GetEnv(), openravepy_int.CollisionOptions.ActiveDOFs):
            try:
                traj = base_manip.MoveHandStraight(direction, minsteps=10 * steps, maxsteps=steps, steplength=step_length,
                                                   ignorefirstcollision=None, starteematrix=None, greedysearch=True, execute=False, outputtraj=None, maxdeviationangle=None,
                                                   planner=None, outputtrajobj=True)
                return list(sample_manipulator_trajectory(manipulator, traj))
            except planning_error:
                return None


def aabb_min(aabb): return aabb.pos() - aabb.extents()


def aabb_max(aabb): return aabb.pos() + aabb.extents()


class AASurface(object):

    def __init__(self, name, xy_min, xy_max, z, color=(0, 0, 0, .25)):
        self.xy_min = xy_min
        self.xy_max = xy_max
        self.z = z
        self.name = name
        self.color = color

    def supports(self, body):
        aabb = body.ComputeAABB()
        return np.all(self.xy_min <= aabb_min(aabb)[:2]) and np.all(aabb_max(aabb)[:2] <= self.xy_max) and abs(aabb_min(aabb)[2] - self.z) < 1e-6

    def sample_placement(self, body, max_attempts=10):
        with body.CreateKinBodyStateSaver():
            for _ in irange(0, max_attempts):
                quat = quat_from_axis_angle(0, 0, uniform(0, 2 * math.pi))
                set_quat(body, quat)
                aabb = body.ComputeAABB()
                low = self.xy_min + aabb.extents()[:2]
                high = self.xy_max - aabb.extents()[:2]
                if np.any(low >= high):
                    continue
                xy = (high - low) * np.random.rand(*low.shape) + low
                z = self.z + aabb.extents()[2]
                point = np.concatenate([xy, [z]]) + \
                    (get_point(body) - aabb.pos())
                pose = pose_from_quat_point(quat, point)

                return pose

        return None

    def draw(self, env):
        self.draw_handle = env.drawtrimesh(points=np.array((
            (self.xy_min[0], self.xy_min[1],
             self.z), (self.xy_min[0], self.xy_max[1], self.z),
            (self.xy_max[0], self.xy_max[1], self.z), (self.xy_max[0], self.xy_min[1], self.z))),
            indices=np.array(((0, 1, 2), (0, 3, 2)), np.int64), colors=np.array((self.color)))

    def __repr__(self):
        return '%s(%s)' % (self.__class__.__name__, self.name)


def compute_surface(body):
    aabb = body.ComputeAABB()
    return AASurface(get_name(body),
                     aabb_min(aabb)[:2],
                     aabb_max(aabb)[:2],
                     aabb_max(aabb)[2] + SURFACE_Z_OFFSET)
