import os, sys, json
import numpy as np
import math
import h5py 
from scipy.spatial.transform import Rotation as R
from enum import Enum, unique
from scipy.signal import savgol_filter

base_path = os.getcwd()
sys.path.append(base_path)
sys.path.append(os.path.join(base_path, "utils"))

from euler_angles.coord_transform_utils import calculation_rotate_matrix, calculation_quaternion, normalize
from pprint import pprint


@unique
class Joints(Enum):
    Hips = "Hips"
    LeftUpLeg = "LeftUpLeg"
    LeftLeg = "LeftLeg"
    LeftFoot = "LeftFoot"
    RightUpLeg = "RightUpLeg"
    RightLeg = "RightLeg"
    RightFoot = "RightFoot"
    Spine = "Spine"
    Spine1 = "Spine1"
    Neck = "Neck"
    LeftShoulder = "LeftShoulder"
    LeftArm = "LeftArm"
    LeftForeArm = "LeftForeArm"
    RightShoulder = "RightShoulder"
    RightArm = "RightArm"
    RightForeArm = "RightForeArm"


CONN_MAP = {
    Joints.Hips: (0, 4),
    Joints.LeftUpLeg: (0, 4),
    Joints.LeftLeg: (4, 5),
    Joints.LeftFoot: (5, 6),
    Joints.RightUpLeg: (0, 1),
    Joints.RightLeg: (1, 2),
    Joints.RightFoot: (2, 3),
    Joints.Spine: (0, 7),
    Joints.Spine1: (7, 8),
    Joints.Neck: (9, 10),
    Joints.LeftShoulder: (8, 11),
    Joints.LeftArm: (11, 12),
    Joints.LeftForeArm: (12, 13),
    Joints.RightShoulder: (8, 14),
    Joints.RightArm: (14, 15),
    Joints.RightForeArm: (15, 16)
}

REF_LIMBS_VECTOR = {
    Joints.Hips: [1.0, 0.0, 0.0],
    Joints.LeftUpLeg: [1.0, 0.0, 0.0],  # 0 LeftUpLeg
    Joints.LeftLeg: [0.0, -1.0, 0.0],  # 1 LeftLeg
    Joints.LeftFoot: [0.0, -1.0, 0.0],  # 2 LeftFoot
    Joints.RightUpLeg: [-1.0, 0.0, 0.0],  # 4 RightUpLeg
    Joints.RightLeg: [0.0, -1.0, 0.0],  # 5 RightLeg
    Joints.RightFoot: [0.0, -1.0, 0.0],  # 6 RightFoot
    Joints.Spine: [0.0, 1.0, 0.0],  # 8 Spine, Hip to Spine
    Joints.Spine1: [0.0, 1.0, 0.0],  # 9 Spine1(Thorax), Spine to Thorax
    Joints.Neck: [0.0, 1.0, 0.0],  # 10 Neck
    Joints.LeftShoulder: [1.0, 0, 0.0],  # 12 LeftShoulder
    Joints.LeftArm: [1.0, 0, 0.0],  # 13 LeftArm
    Joints.LeftForeArm: [1.0, 0, 0.0],  # 14 LeftForeArm
    Joints.RightShoulder: [-1.0, 0.0, 0.0],  # 18 RightShoulder
    Joints.RightArm: [-1.0, 0.0, 0.0],  # 19 RightArm
    Joints.RightForeArm: [-1.0, 0.0, 0.0],  # 20 RightForeArm
}

PARENT_INFO = {
    Joints.LeftLeg: None,  # 1
    Joints.LeftFoot: [Joints.LeftLeg],  # 2
    Joints.RightLeg: None,  # 5
    Joints.RightFoot: [Joints.RightLeg],  # 6
    Joints.Spine: None,  # 8
    Joints.Spine1: [Joints.Spine],  # 9

    Joints.LeftArm: None,  # 13
    Joints.LeftForeArm: [Joints.LeftArm],  # 14

    Joints.RightArm: None,  # 19
    Joints.RightForeArm: [Joints.RightArm, ],  # 20
}


def pre_processing(inp_3d_coords):

    processed_coords = np.reshape(inp_3d_coords, (-1, 3))

    center_hip_coord = [(processed_coords[0, 0] + processed_coords[3, 0]) / 2,
                        (processed_coords[0, 1] + processed_coords[3, 1]) / 2,
                        (processed_coords[0, 2] + processed_coords[3, 2]) / 2]

    processed_coords = np.insert(processed_coords, 0, center_hip_coord, 0)

    neck = [(processed_coords[11, 0] + processed_coords[14, 0]) / 2,
            (processed_coords[11, 1] + processed_coords[14, 1]) / 2,
            (processed_coords[11, 2] + processed_coords[14, 2]) / 2]

    processed_coords[8, :] = neck

    kp, _ = processed_coords.shape

    # ensure right hand coordinate
    for j in range(kp):
        processed_coords[j, 1] = -processed_coords[j, 1]
        processed_coords[j, 2] = -processed_coords[j, 2]

    return processed_coords

kpt_queue = []
def smooth_filter(kpts):
    if len(kpt_queue) < 6:
        kpt_queue.append(kpts)
        return kpts[0]

    queue_length = len(kpt_queue)
    if queue_length == 50:
        kpt_queue.pop(0)
    kpt_queue.append(kpts)

    transKpts = np.array(kpt_queue).transpose(1, 2, 3, 0)

    window_length = queue_length - 1 if queue_length % 2 == 0 else queue_length - 2
    # array, window_length(bigger is better), polyorder
    result = savgol_filter(transKpts, window_length, 3).transpose(3, 0, 1, 2) 
    aa = result[window_length // 2][0]
    return result[window_length // 2][0]


def cal_transform(processed_coords):
    # hip rotation
    s, e = CONN_MAP[Joints.Hips]
    hip_res_vec = processed_coords[e, :] - processed_coords[s, :]
    # Cancel y offset
    hip_res_vec[1] = 0

    hip_ori_vec = REF_LIMBS_VECTOR[Joints.Hips]
    hip_rotate = list(calculation_rotate_matrix(hip_ori_vec, hip_res_vec))
    hip_quaternion = list(calculation_quaternion(hip_ori_vec, hip_res_vec))

    rot_cache = {}
    aa = {}
    for name, conn in CONN_MAP.items():
        s, e = conn
        res_vector = processed_coords[e, :] - processed_coords[s, :]
        ori_vector = np.array(REF_LIMBS_VECTOR[name])

        res_vector = hip_rotate[0].T @ res_vector

        res_vector /= normalize(res_vector)
        ori_vector /= normalize(ori_vector)

        rot_m, _ = calculation_rotate_matrix(ori_vector, res_vector)
        rot_cache[name] = rot_m 

        if PARENT_INFO.get(name, None) is not None:
            par = PARENT_INFO[name][-1]
            res_vector = rot_cache[par].T @ res_vector
        quaternion = calculation_quaternion(ori_vector, res_vector)

        aa[name] = {
            "x": -quaternion[0],
            "y": quaternion[1],
            "z": -quaternion[2],
            "w": quaternion[3]
        }

    aa[Joints.Hips] = {
        "x": -hip_quaternion[0],
        "y": hip_quaternion[1],
        "z": -hip_quaternion[2],
        "w": hip_quaternion[3]
    }

    return aa


def angle_calculate(x, y):
    cos_angle = x.dot(y)/(np.linalg.norm(x) * np.linalg.norm(y)) 
    return np.arccos(cos_angle) 


def pose_calculation(specifiy_coords) :
    
    frame_comut = specifiy_coords.shape[0]
    pos_frame = {}
    pos_frame['frame_count'] = frame_comut
    for i in range(frame_comut):
        angle_x = -90
        angle_y = 180
        angle_z = -90
        pos_tmp = specifiy_coords[i, :, :]
        pos_tmp1 = pos_tmp[np.newaxis, :]
        pos = smooth_filter(pos_tmp1)
        rx = np.array([[1, 0, 0], [0, np.cos(math.radians(angle_x)), np.sin(math.radians(angle_x))], [0, -np.sin(math.radians(angle_x)), np.cos(math.radians(angle_x))]])
        ry = np.array([[np.cos(math.radians(angle_y)), 0, -np.sin(math.radians(angle_y))], [0, 1, 0], [np.sin(math.radians(angle_y)), 0, np.cos(math.radians(angle_y))]])
        rz = np.array([[np.cos(math.radians(angle_z)), np.sin(math.radians(angle_z)), 0], [-np.sin(math.radians(angle_z)), np.cos(math.radians(angle_z)), 0], [0, 0, 1]])
        pos = np.dot(np.dot(pos, rx), ry)

        specifiy_quaternion = cal_transform(pos)

        our_map = {"Hips": Joints.Hips,         "LeftUpLeg": Joints.LeftLeg,    "RightUpLeg": Joints.RightLeg,     "Spine": None,
                   "LeftLeg": Joints.LeftFoot,  "RightLeg": Joints.RightFoot,   "Spine1": None,                    "LeftFoot": None,
                   "RightFoot": None,           "Spine2": None,                 "LeftToeBase": None,               "RightToeBase": None, 
                   "Neck": None,                "LeftShoulder": None,           "RightShoulder": None,             "Head": None,
                   "LeftArm": Joints.LeftArm,   "RightArm": Joints.RightArm,    "LeftForeArm": Joints.LeftForeArm, "RightForeArm": Joints.RightForeArm,
                   "LeftHand": None,            "RightHand": None,              "LeftHandIndex1": None,            "RightHandIndex1": None}

        cur = {
            "code": 0,
            "function": 1,
            "translation": {
            "x": 0,
            "y": 0,
            "z": 0
            },
            "data": {},
        }

        keyquaternion = {"keyquaternions": []}

        hip_rotate = None
        for k, v in our_map.items():
            if v is None:
                quaternion = {
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "w": 1,
                }
            else:
                quaternion = specifiy_quaternion[v].copy()

            aa = {}
            aa['name'] = k
            aa['quaternion'] = quaternion
            keyquaternion['keyquaternions'].append(aa)
        cur['data'] = keyquaternion
        pos_frame['frame{}'.format(i)] = cur

    with open("body_quaternion.json".format(i), 'w') as json_file:
        json.dump(pos_frame, json_file, indent=4)


if __name__ == '__main__':
    file_name = os.path.dirname(__file__) +'/body_coordinate.h5'
    body_quaternion = h5py.File(file_name,'r')
    specifiy_coords = body_quaternion['data'][:]  
    pose_calculation(specifiy_coords)
