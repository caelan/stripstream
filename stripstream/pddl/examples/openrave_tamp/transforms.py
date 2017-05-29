from misc.utils import *
from openravepy import poseTransformPoints, matrixFromPose, matrixFromQuat, matrixFromAxisAngle, rotationMatrixFromQuat,  quatFromAxisAngle, poseFromMatrix, axisAngleFromRotationMatrix, quatFromRotationMatrix, quatMult, quatInverse,  quatRotateDirection, quatSlerp, RaveGetAffineDOFValuesFromTransform, DOFAffine, transformLookat
import numpy as np
from math import sin, cos


def norm(vector, order=2):
    return np.linalg.norm(vector, ord=order)


def length(vector):
    return np.linalg.norm(vector)


def length2(vector):
    return np.dot(vector, vector)


def normalize(vector):
    return 1. / length(vector) * vector


def unit_x():
    return np.array((1, 0, 0))


def unit_y():
    return np.array((0, 1, 0))


def unit_z():
    return np.array((0, 0, 1))


def unit_point():
    return np.zeros(3)


def unit_quat():
    return np.array((1, 0, 0, 0))


def unit_rot():
    return np.identity(3)


def unit_pose():
    return np.array((1, 0, 0, 0, 0, 0, 0))


def unit_trans():
    return np.identity(4)


def trans_transform_point(trans, point):
    return trans.dot(np.concatenate([point, [1]]).T)[:3]


def trans_transform_points(trans, points):
    return np.dot(trans[:3, :3], points) + np.tile(trans[:3, 3].T, (points.shape[1], 1)).T


def pose_transform_point(pose, point):
    return poseTransformPoints(pose, np.array([point]))[0]


def quat_transform_point(quat, point):
    return pose_transform_point(pose_from_quat_point(quat, unit_point()), point)


def rot_transform_point(rot, point):
    return rot.dot(point)


def quat_from_pose(pose):
    return pose[:4]


def point_from_pose(pose):
    return pose[4:]


def pose_from_quat_point(quat, point):
    return np.concatenate([quat, point])


def trans_from_quat_point(quat, point):
    return trans_from_pose(pose_from_quat_point(quat, point))


def trans_from_pose(pose):
    return matrixFromPose(pose)


def pose_from_trans(trans):
    return poseFromMatrix(trans)


def trans_from_point(x, y, z):
    return trans_from_quat_point(unit_quat(), np.array([x, y, z]))


def trans_from_quat(quat):
    return matrixFromQuat(quat)


def trans_from_axis_angle(x_angle, y_angle, z_angle):
    return trans_from_quat(quat_from_axis_angle(x_angle, y_angle, z_angle))


def trans_from_rot_point(rot, point):
    trans = unit_trans()
    trans[:3, :3] = rot
    trans[:3, 3] = point
    return trans


def trans_from_rot(rot):
    return trans_from_rot_point(rot, unit_point())


def rot_from_trans(trans):
    return trans[:3, :3]


def quat_from_trans(trans):
    return quat_from_rot(rot_from_trans(trans))


def point_from_trans(trans):
    return trans[:3, 3]


def quat_from_axis_angle(x_angle, y_angle, z_angle):
    return quatFromAxisAngle(np.array((x_angle, y_angle, z_angle)))


def rot_from_axis_angle(x_angle, y_angle, z_angle):
    return matrixFromAxisAngle(np.array((x_angle, y_angle, z_angle)))


def axis_angle_from_rot(rot):
    return axisAngleFromRotationMatrix(rot)


def quat_from_rot(rot):
    return quatFromRotationMatrix(rot)


def rot_from_quat(quat):
    return rotationMatrixFromQuat(quat)


def quat_from_angle_vector(angle, vector):
    return np.concatenate([[cos(angle / 2)], sin(angle / 2) * normalize(np.array(vector))])


def rot_from_angle_vector(angle, vector):
    return rot_from_quat(quat_from_angle_vector(angle, vector))


def trans_dot(*trans):
    return np.dot(*trans)


def trans_inv(trans):
    return np.linalg.inv(trans)


def quat_dot(*quats):
    return reduce(quatMult, quats)


def quat_inv(quat):
    return quatInverse(quat)


def quat_look_at(vector1, vector2=None):
    if vector2 is None:
        vector2 = vector1
        vector1 = unit_x()
    return quatRotateDirection(vector1, vector2)


def rot_look_at(vector1, vector2=None):
    return rot_from_quat(quat_look_at(vector1, vector2))


def camera_look_at(point, look_point=unit_point()):
    return transformLookat(np.array(look_point) - np.array(point), np.array(point), -unit_z())


def quat_interpolate(quat1, quat2, t=.5):
    return quatSlerp(quat1, quat2, t, True)


def pose_interpolate(pose1, pose2, t=.5):
    return pose_from_quat_point(quat_interpolate(quat_from_pose(pose1), quat_from_pose(pose2), t),
                                t * point_from_pose(pose1) + (1 - t) * point_from_pose(pose2))


def vector_trans(trans, vector):
    return trans_from_pose(pose_from_trans(trans) + np.concatenate([np.zeros((4,)), vector]))


def quat_from_z_rot(theta):
    return quat_from_axis_angle(0, 0, theta)


def base_values_from_trans(trans):
    return RaveGetAffineDOFValuesFromTransform(trans, DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])


def xyzt_from_trans(trans):
    return RaveGetAffineDOFValuesFromTransform(trans, DOFAffine.X | DOFAffine.Y | DOFAffine.Z | DOFAffine.RotationAxis, [0, 0, 1])


def base_values_from_pose(pose):
    return base_values_from_trans(trans_from_pose(pose))


def is_upright(trans):
    return within(abs(trans[2, 2]), 1)


def pose_from_base_values(base_values, z=0.0):
    x, y, theta = base_values
    return pose_from_quat_point(quat_from_z_rot(theta), [x, y, z])


def trans_from_base_values(base_values, z=0.0):
    return trans_from_pose(pose_from_base_values(base_values, z=z))


def trans2D_from_trans(trans):
    return trans_from_base_values(base_values_from_trans(trans))


def point_from_full_config(config):
    return config[-7:-4]


def quat_from_full_config(config):
    return config[-4:]


def pose_from_full_config(config):
    return np.concatenate([quat_from_full_config(config), point_from_full_config(config)])


def trans_from_full_config(config):
    return trans_from_pose(pose_from_full_config(config))


def base_values_from_full_config(config):
    return base_values_from_pose(pose_from_full_config(config))


def arm_from_full_config(arm, config):
    return config[arm.GetArmIndices()]


def arm_and_base_from_full_config(arm, config):
    return np.concatenate([arm_from_full_config(arm, config), base_values_from_full_config(config)])


def full_config_from_pose(pose, config):
    new_config = config.copy()
    new_config[-7:-4] = pose[-3:]
    new_config[-4:] = pose[:4]
    return new_config


def full_config_from_trans(trans, config):
    return full_config_from_pose(pose_from_trans(trans), config)


def full_config_from_base_values(base_values, config):
    _, _, z = point_from_full_config(config)
    return full_config_from_pose(pose_from_base_values(base_values, z=z), config)


def get_trans(body):
    return body.GetTransform()


def get_pose(body):
    return pose_from_trans(get_trans(body))


def get_point(body):
    return point_from_trans(get_trans(body))


def get_quat(body):
    return quat_from_pose(get_pose(body))


def get_config(body, joint_indices=None):
    if joint_indices is None:
        return body.GetDOFValues()
    return body.GetDOFValues(indices=joint_indices)


def get_active_config(body):
    return body.GetActiveDOFValues()


def get_active_indices(body):
    return body.GetActiveDOFIndices()


def get_full_config(body, dof_indices=None):
    if dof_indices is None:
        return body.GetConfigurationValues()
    return body.GetConfigurationValues()[dof_indices]


def set_trans(body, trans):
    body.SetTransform(trans)


def set_pose(body, pose):
    set_trans(body, trans_from_pose(pose))


def set_xy(body, x, y):
    point = get_point(body)
    set_point(body, np.array([x, y, point[2]]))


def set_point(body, point):
    set_pose(body, pose_from_quat_point(get_quat(body), point))


def set_quat(body, quat):
    set_pose(body, pose_from_quat_point(quat, get_point(body)))


def set_config(body, config, joint_indices=None):
    if joint_indices is None:
        body.SetDOFValues(config)
    else:
        body.SetDOFValues(config, joint_indices)


def set_active_config(body, config):
    body.SetActiveDOFValues(config)


def set_active_indices(body, indices):
    body.SetActiveDOFs(indices)


def set_full_config(body, config, dof_indices=None):
    if dof_indices is None:
        body.SetConfigurationValues(config)
    else:
        full_config = get_full_config(body)
        full_config[dof_indices] = config
        body.SetConfigurationValues(full_config)


def set_base_values(body, base_values):
    trans = get_trans(body)
    trans[:3, :3] = rot_from_quat(quat_from_z_rot(base_values[-1]))
    trans[:2, 3] = base_values[:2]
    set_trans(body, trans)


def set_manipulator_values(manipulator, values):
    set_config(manipulator.GetRobot(), values, manipulator.GetArmIndices())


def object_trans_from_manip_trans(manip_trans, grasp):
    return np.dot(manip_trans, grasp)


def manip_trans_from_object_trans(object_trans, grasp):

    return np.linalg.solve(grasp.T, object_trans.T).T


def compute_grasp(manip_trans, object_trans):
    return np.linalg.solve(manip_trans, object_trans)
