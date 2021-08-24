import cv2
import math
import numpy as np

from scipy.spatial.transform import Rotation as R


def dot_product(vec1, vec2):
    return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]


def cross_product(vec1, vec2):
    x = vec1[1] * vec2[2] - vec1[2] * vec2[1]
    y = vec1[2] * vec2[0] - vec1[0] * vec2[2]
    z = vec1[0] * vec2[1] - vec1[1] * vec2[0]
    return [x, y, z]


def normalize(vec):
    return math.sqrt(dot_product(vec, vec))


def length_calculation(s, e):
    """
    s = (x, y, z)
    e = (x1, y1, z1)
    :param s:
    :param e:
    :return:
    """
    sum_res = (s[0] - e[0]) ** 2 + (s[1] - e[1]) ** 2 + (s[2] - e[2]) ** 2
    return math.sqrt(sum_res)


def rotation_matrix(angle, axis):
    """
    :param angle:
    :param axis:
    :return:
    """
    norm = normalize(axis)

    axis[0] /= norm
    axis[1] /= norm
    axis[2] /= norm

    rotate_matrix = np.zeros((3, 3), dtype=np.float32)

    rotate_matrix[0, 0] = math.cos(angle) + axis[0] * axis[0] * (1 - math.cos(angle))
    rotate_matrix[0, 1] = axis[0] * axis[1] * (1 - math.cos(angle) - axis[2] * math.sin(angle))
    rotate_matrix[0, 2] = axis[1] * math.sin(angle) + axis[0] * axis[2] * (1 - math.cos(angle))

    rotate_matrix[1, 0] = axis[2] * math.sin(angle) + axis[0] * axis[1] * (1 - math.cos(angle))
    rotate_matrix[1, 1] = math.cos(angle) + axis[1] * axis[1] * (1 - math.cos(angle))
    rotate_matrix[1, 2] = -axis[0] * math.sin(angle) + axis[1] * axis[2] * (1 - math.cos(angle))

    rotate_matrix[2, 0] = -axis[1] * math.sin(angle) + axis[0] * axis[2] * (1 - math.cos(angle))
    rotate_matrix[2, 1] = axis[0] * math.sin(angle) + axis[1] * axis[2] * (1 - math.cos(angle))
    rotate_matrix[2, 2] = math.cos(angle) + axis[2] * axis[2] * (1 - math.cos(angle))

    return rotate_matrix


def rotation_matrix2(axis, theta):
    # Return the rotation matrix associated with counterclockwise rotation about
    # the given axis by theta radians.

    if theta == 0:
        return np.eye(3, 3)
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d

    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    rot_mat = np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

    return rot_mat


def calculation_rotate_matrix(vector_before, vector_after):
    """
    :param vector_before:
    :param vector_after:
    :return: matrix is based on unit vector before and after
    """
    axis = cross_product(vector_before, vector_after)
    angle = math.acos(dot_product(vector_before, vector_after) / normalize(vector_before) / normalize(vector_after))
    # rotate_matrix = rotation_matrix(angle, axis)
    rotate_matrix = rotation_matrix2(axis, angle)
    return rotate_matrix, (axis, math.degrees(angle))

def calculation_quaternion(vector_before, vector_after):
    """
    :param vector_before:
    :param vector_after:
    :return: quaternion is based on unit vector before and after
    """
    axis = cross_product(vector_before, vector_after)
    aa = np.linalg.norm(np.array(axis))
    if aa < 1.0e-5:
        return [0, 0, 0, 1]
    angle = math.acos(dot_product(vector_before, vector_after) / normalize(vector_before) / normalize(vector_after))
    aa = [axis, angle]
    quaternion = axis_angle_to_quaternion(aa)
    return quaternion




def calculate_unit_vector(st_coord, end_coord):
    len = math.sqrt((end_coord[0] - st_coord[0]) ** 2 + (end_coord[1] - st_coord[1]) ** 2 + (
            end_coord[2] - st_coord[2]) ** 2)
    vec = [(end_coord[0] - st_coord[0]) / len, (end_coord[1] - st_coord[1]) / len, (end_coord[2] - st_coord[2]) / len]
    return vec


def convert_to_xyz(xy_coords, gray_img, x_offset=0, y_offset=0, inverse_axis=False):
    """
    :param xy_coords: [(x1, y1), (x2, y2), (x3, y3)...]
    :param gray_img:
    :return:
    """
    ref_width = 2316
    ref_height = 3088
    cam_ox = 1153.142
    cam_oy = 1537.5546
    cam_fx = 2884.3987
    cam_fy = 2884.3987

    gray_img = cv2.cvtColor(gray_img, cv2.COLOR_BGR2GRAY)

    xs = []
    ys = []
    zs = []
    ori_zs = []
    for i, (x, y) in enumerate(xy_coords):
        x = min(x, 479)
        y = min(y, 639)
        if i == 0:
            y += xy_coords[1][1]
            y //= 2

        z = gray_img[y, x] / 51 * 100
        ori_zs.append(z)
        new_x = ((x / 480) * ref_width - cam_ox) * z / cam_fx + x_offset
        new_y = ((y / 640) * ref_height - cam_oy) * z / cam_fy + y_offset
        xs.append(new_x)
        ys.append(new_y)
        zs.append(z)

    z_center = zs[1]
    mul_factor = 1.0
    if inverse_axis:
        mul_factor = -1.0
    zs = list(map(lambda p: (p - z_center) * mul_factor * 0.6 + 50, zs))
    x_center = xs[1]
    xs = list(map(lambda p: (p - x_center) * mul_factor * 0.6 + 50, xs))
    # y_center = ys[1]
    # ys = list(map(lambda p: -(p - y_center) * 0.6 + 80, ys))
    ys = list(map(lambda p: -p * 0.6 + 50, ys))

    return {
        "xs": xs,
        "ys": ys,
        "zs": zs,
        "ori_zs": ori_zs
    }


def euler_to_rotation_matrix2(x_degree, y_degree, z_degree):
    x_cos = math.cos(math.radians(x_degree))
    x_sin = math.sin(math.radians(x_degree))

    y_cos = math.cos(math.radians(y_degree))
    y_sin = math.sin(math.radians(y_degree))

    z_cos = math.cos(math.radians(z_degree))
    z_sin = math.sin(math.radians(z_degree))

    x_rot_m = np.array([[1, 0, 0],
                        [0, x_cos, -x_sin],
                        [0, x_sin, x_cos]])

    y_rot_m = np.array([[y_cos, 0, y_sin],
                        [0, 1, 0],
                        [-y_sin, 0, y_cos]])

    z_rot_m = np.array([[z_cos, -z_sin, 0],
                        [z_sin, z_cos, 0],
                        [0, 0, 1]])

    tmp = x_rot_m * y_rot_m
    final_m = tmp * z_rot_m
    return final_m


def axis_angle_to_quaternion(aa):
    x, y, z = aa[0]
    degree = aa[1]
    # radian_degree = math.radians(degree / 2) # 弧度转化
    radian_degree = degree / 2
    qx = x * math.sin(radian_degree)
    qy = y * math.sin(radian_degree)
    qz = z * math.sin(radian_degree)
    qw = math.cos(radian_degree)
    return [qx, qy, qz, qw]


def euler_to_axis_angle(euler_angles, order="XYZ"):
    r = R.from_euler(order, euler_angles, degrees=True)
    rot_vec = r.as_rotvec()
    radian = np.sum(rot_vec ** 2) ** 0.5
    if radian == 0:
        return [0, 1, 0, 0]
    rot_vec /= radian
    degree = radian / np.pi * 180
    return [rot_vec[0], rot_vec[1], rot_vec[2], degree]


def check_nan(x):
    if x == math.inf or x == -math.inf or str(x) == "nan":
        return 0
    return x


def get_proj(self):
    """
    Create the projection matrix from the current viewing position.

    elev stores the elevation angle in the z plane
    azim stores the azimuth angle in the x,y plane

    dist is the distance of the eye viewing point from the object
    point.

    """
    relev, razim = np.pi * self.elev/180, np.pi * self.azim/180

    xmin, xmax = self.get_xlim3d()
    ymin, ymax = self.get_ylim3d()
    zmin, zmax = self.get_zlim3d()

    # transform to uniform world coordinates 0-1.0,0-1.0,0-1.0
    worldM = proj3d.world_transformation(xmin, xmax,
                                            ymin, ymax,
                                            zmin, zmax)

    # look into the middle of the new coordinates
    R = np.array([0.5, 0.5, 0.5])

    xp = R[0] + np.cos(razim) * np.cos(relev) * self.dist
    yp = R[1] + np.sin(razim) * np.cos(relev) * self.dist
    zp = R[2] + np.sin(relev) * self.dist
    E = np.array((xp, yp, zp))

    self.eye = E
    self.vvec = R - E
    self.vvec = self.vvec / np.linalg.norm(self.vvec)

    if abs(relev) > np.pi/2:
        # upside down
        V = np.array((0, 0, -1))
    else:
        V = np.array((0, 0, 1))
    zfront, zback = -self.dist, self.dist

    viewM = proj3d.view_transformation(E, R, V)
    projM = self._projection(zfront, zback)
    M0 = np.dot(viewM, worldM)
    M = np.dot(projM, M0)
    return M
