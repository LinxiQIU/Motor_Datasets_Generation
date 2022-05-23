import bpy
import os
import blensor
import math
import random
import csv
import numpy as np
import sys
import argparse
from get_3d_bbox import findpoints, motor_points, \
    transfer_cam2obj, transfer_obj2cam, deflect, rectify
from points2pcd import points2pcd


def get_ObjectID(x):  # get all kinds of ObjectID from numpy file

    dic = []
    for i in range(x.shape[0]):
        if x[i][8] not in dic:
            dic.append(x[i][8])

    return dic


def ChangeLabel(x):
    if x.shape[1] == 13:
        for i in range(x.shape[0]):
            if x[i][8] == 808464432.0:
                x[i][8] = int(0)
            elif x[i][8] == 825307441.0:
                x[i][8] = int(1)
            elif x[i][8] == 842150450.0:
                x[i][8] = int(2)
            elif x[i][8] == 858993459.0:
                x[i][8] = int(3)
            elif x[i][8] == 875836468.0:
                x[i][8] = int(4)
            elif x[i][8] == 892679477.0:
                x[i][8] = int(5)
            elif x[i][8] == 909522486.0:
                x[i][8] = int(6)
            elif x[i][8] == 926365495.0:
                x[i][8] = int(7)
    else:
        print("The cor of numpy is not right")
    return x


def Resort_IDX(x):  # reset the IDX Value in the filtered numpy

    for i in range(x.shape[0]):
        x[i][-1] = i

    return x


def cutNumpy(x):  # drop the timestamp, yaw, pitch off and the point of (0,0,0)

    try:
        if x.shape[1] == 16:
            x = x[:, 3:]
    except Exception as err:
        print(err)
    # Filter all points with a distance along the z coordinate small than 0
    y = x[x[:, 7] < 0]

    return y


def initial_clamp(Clamping_dir):
    add_plane()
    objects_name = []
    for obj in bpy.data.objects:
        objects_name.append(obj.name)

    if not '0000_Clamping' in objects_name:
        import_ClampingSystem_obj(Clamping_dir)


def import_MotorPart_obj(path, filters):
    need_file_items = []
    need_file_names = []

    filterDict = {}
    for item in filters:
        filterDict[item] = True

    file_lst = os.listdir(path)

    for item in file_lst:
        fileName, fileExtension = os.path.splitext(item)
        if fileExtension == ".obj" and (not item in filterDict):
            need_file_items.append(item)
            need_file_names.append(fileName)

    n = len(need_file_items)
    for i in range(n):
        item = need_file_items[i]
        itemName = need_file_names[i]
        ufilename = path + "\\" + item
        bpy.ops.import_scene.obj(filepath=ufilename, filter_glob="*.obj")
    rename_element('Motor')
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.scene.objects.active = None
    resize_element('Motor')


def import_ClampingSystem_obj(path):  # for Blender 2.79-Version: need rename and conbination

    bpy.ops.import_scene.obj(filepath=path + '\\clamp_counterpart.obj', filter_glob="*.obj")
    bpy.ops.import_scene.obj(filepath=path + '\\foundation_slider.obj', filter_glob="*.obj")
    bpy.ops.import_scene.obj(filepath=path + '\\operator_panel.obj', filter_glob="*.obj")
    bpy.ops.import_scene.obj(filepath=path + '\\pillow.obj', filter_glob="*.obj")
    bpy.ops.import_scene.obj(filepath=path + '\\plate.obj', filter_glob="*.obj")
    bpy.ops.import_scene.obj(filepath=path + '\\plc_enclosure.obj', filter_glob="*.obj")
    bpy.ops.import_scene.obj(filepath=path + '\\pneumatic.obj', filter_glob="*.obj")
    bpy.ops.import_scene.obj(filepath=path + '\\slider.obj', filter_glob="*.obj")
    bpy.ops.import_scene.obj(filepath=path + '\\Cylinder.obj', filter_glob="*.obj")
    rename_element('Clamping')
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.scene.objects.active = None
    resize_element('Clamping')


def rename_element(element_type):  # element_type = ['Clamping', 'Motor']
    k = 0
    if element_type == 'Clamping':
        for i in range(len(bpy.data.objects)):
            print(bpy.data.objects[i].name)
            if 'Cylinder' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '0000_Clamping_' + 'Cylinder'
            if 'PLC' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '0000_Clamping_' + 'plc_enclosure'
            if 'Left_fixed' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '0000_Clamping_' + 'foundation_left_clamp'
            if 'Clamping_Right' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '0000_Clamping_' + 'countpart_right_clamp'
            if 'Slider_part' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '0000_Clamping_' + 'Slider'
            if 'Pneumatic_Assembly' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '0000_Clamping_' + 'Pneumatic'
            if 'Plate' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '0000_Clamping_' + 'plate'
            if 'operator_panel&duct' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '0000_Clamping_' + 'operator_panel'
            if 'MATRIX_Prototyp' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '0000_Clamping_' + 'pillar'

    elif element_type == 'Motor':
        for i in range(len(bpy.data.objects)):
            if ('Bolt_0' in bpy.data.objects[i].name) or ('Bolt_1' in bpy.data.objects[i].name):
                bpy.data.objects[i].name = '5555_side_bolt_' + str(k)
                k += 1
            elif 'Bolt' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '6666_cover_bolt_' + str(k)
                k += 1
            elif 'Bottom' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '4444_Bottom'
            elif 'Charge' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '3333_Charger'
            elif 'Cover' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '1111_Cover'
            elif 'Gear_Container' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '2222_Gear_Container'
            elif 'Inner_Gear' in bpy.data.objects[i].name:
                bpy.data.objects[i].name = '1111_Inner_Gear'


def initialize_lamp():
    bpy.ops.object.select_all(action='DESELECT')

    if bpy.data.objects['Point'] and bpy.data.objects['Point.001'] and bpy.data.objects['Point.002']:
        bpy.data.objects['Point'].select = True
        bpy.context.scene.objects.active = bpy.data.objects['Point']
        bpy.context.object.data.energy = 0.8
        bpy.context.object.data.use_specular = False

        bpy.ops.object.select_all(action='DESELECT')

        bpy.data.objects['Point.001'].select = True
        bpy.context.scene.objects.active = bpy.data.objects['Point.001']
        bpy.context.object.data.energy = 0.8
        bpy.context.object.data.use_specular = False

        bpy.ops.object.select_all(action='DESELECT')

        bpy.data.objects['Point.002'].select = True
        bpy.context.scene.objects.active = bpy.data.objects['Point.002']
        bpy.context.object.data.energy = 0.8
        bpy.context.object.data.use_specular = False

        bpy.ops.object.select_all(action='DESELECT')


# Add a plane as the background
def add_plane():
    bpy.ops.mesh.primitive_plane_add(view_align=False, enter_editmode=False, location=(0, 0, 0),
                                     layers=(
                                         True, False, False, False, False, False, False, False, False, False, False,
                                         False,
                                         False, False, False, False, False, False, False, False))
    bpy.data.objects['Plane'].name = '0000_Plane'
    bpy.ops.object.select_all(action='DESELECT')
    bpy.data.objects['0000_Plane'].select = True
    bpy.context.scene.objects.active = bpy.data.objects['0000_Plane']
    bpy.ops.transform.resize(value=(3, 3, 3), constraint_axis=(False, False, False), constraint_orientation='GLOBAL',
                             mirror=False,
                             proportional='DISABLED', proportional_edit_falloff='SMOOTH', proportional_size=1)
    bpy.ops.transform.translate(value=(0, 0, 0), constraint_axis=(False, False, False),
                                constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                proportional_edit_falloff='SMOOTH', proportional_size=1)
    bpy.ops.object.select_all(action='DESELECT')


def create_csv(csv_path):
    if not os.path.exists(csv_path):
        os.makedirs(csv_path)
    csv_path = csv_path + '\\camera_motor_setting.csv'
    with open(csv_path, 'a+', newline='') as f:
        csv_writer = csv.writer(f)
        head = ["camera_positionX", "camera_positionY", "camera_positionZ", "camera_eulerX",
                "camera_eulerY", "camera_eulerZ", "motor_eulerX", "motor_eulerY", "motor_eulerZ"]
        csv_writer.writerow(head)


def create_bbox_csv(save_dir):
    if not os.path.exists(save_dir):
        os.mkdir(save_dir)
    save_dir = save_dir + '\\motor_3D_bounding_box.csv'
    with open(save_dir, 'a+', newline='') as f:
        csv_writer = csv.writer(f)
        head = ["motor_id", "location_x", "location_x", "location_z", "height", "width", "length", "eulerX", "eulerY",
                "eulerZ"]
        csv_writer.writerow(head)


def read_bottomLength(csv_path):
    bottomLength = []
    with open(csv_path, "r+") as f:
        csv_read = csv.reader(f)
        for line in csv_read:
            if line[13] != 'mf_Bottom_Length':
                bottomLength.append(float(line[13]) * 0.05)
    return bottomLength


def read_subBottomLength(csv_path):
    subBottomLength = []
    with open(csv_path, "r+") as f:
        csv_read = csv.reader(f)
        for line in csv_read:
            if line[14] != 'mf_Sub_Bottom_Length':
                subBottomLength.append(float(line[14]) * 0.05)
    return subBottomLength


def random_CameraPosition(radius_camera):
    r = radius_camera
    theta = [0, 15.0 * math.pi / 180.0]  # 70`~90`
    phi = [0, 2 * math.pi]

    phi_camera = random.uniform(phi[0], phi[1])
    theta_camera = random.uniform(theta[0], theta[1])
    x_camera = r * math.cos(phi_camera) * math.sin(theta_camera)
    y_camera = r * math.sin(phi_camera) * math.sin(theta_camera)
    z_camera = r * math.cos(theta_camera)
    # bpy.data.objects['Camera'].location = (x_camera, y_camera, z_camera)

    bpy.ops.object.select_all(action='DESELECT')
    bpy.data.objects['Camera'].select = True
    bpy.context.scene.objects.active = bpy.data.objects['Camera']

    beta_1 = math.atan2(abs(y_camera), abs(z_camera))
    beta_2 = math.asin(abs(x_camera) / (y_camera ** 2 + z_camera ** 2) ** 0.5)
    # beta_2 = math.asin(abs(x_camera) / (y_camera**2 + z_camera**2)**0.5)
    # rotation_z = -1.57
    z = [-115 * math.pi / 180.0, -65 * math.pi / 180.0]
    rotation_z = random.uniform(z[0], z[1])

    if 0 < phi_camera < math.pi / 2.0:
        rotation_x = -beta_2
        rotation_y = -beta_1
    if math.pi / 2.0 < phi_camera < math.pi:
        rotation_x = beta_2
        rotation_y = -beta_1
    if math.pi < phi_camera < 3.0 * math.pi / 2.0:
        rotation_x = beta_2
        rotation_y = beta_1
    if 3.0 * math.pi / 2.0 < phi_camera < 2.0 * math.pi:  ###  ???
        rotation_x = -beta_2
        rotation_y = beta_1

    if phi_camera == 0.0:
        rotation_x = theta_camera
        rotation_y = 0
    if phi_camera == math.pi / 2.0:
        rotation_x = 0
        rotation_y = theta_camera
    if phi_camera == math.pi:
        rotation_x = -theta_camera
        rotation_y = 0
    if phi_camera == 3.0 * math.pi / 2.0:
        rotation_x = 0
        rotation_y = -theta_camera

    bpy.data.objects['Camera'].rotation_euler = (rotation_x, rotation_y, rotation_z)
    bpy.ops.object.select_all(action='DESELECT')
    # find a position above the motor
    x_camera += (-0.915)
    y_camera += 0.3
    z_camera += 1.16
    bpy.data.objects['Camera'].location = (x_camera, y_camera, z_camera)
    cam_info = [x_camera, y_camera, z_camera, rotation_x, rotation_y, rotation_z]
    return cam_info


def init_cam_position(random_cam_info):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.data.objects['Camera'].select = True
    bpy.context.scene.objects.active = bpy.data.objects['Camera']
    bpy.data.objects['Camera'].rotation_euler = (random_cam_info[3], random_cam_info[4], random_cam_info[5])
    bpy.data.objects['Camera'].location = (random_cam_info[0], random_cam_info[1], random_cam_info[2])
    return random_cam_info


def resize_element(element_type):  # Resize ClampingSystem to 5 scale and Motor 0.05 scale

    factor_clamp = 0.503
    factor_motor = 0.05
    if element_type == 'Clamping':
        for obj in bpy.data.objects:
            if '0000_Clamping_' in obj.name:
                bpy.ops.object.select_all(action='DESELECT')
                obj.select = True
                bpy.ops.transform.resize(value=(factor_clamp, factor_clamp, factor_clamp),
                                         constraint_axis=(False, False, False),
                                         constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                         proportional_edit_falloff='SMOOTH', proportional_size=1)
    elif element_type == 'Motor':
        for obj in bpy.data.objects:
            if ('0000' not in obj.name) and ('Camera' not in obj.name):
                bpy.ops.object.select_all(action='DESELECT')
                obj.select = True
                bpy.ops.transform.resize(value=(factor_motor, factor_motor, factor_motor),
                                         constraint_axis=(False, False, False),
                                         constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                         proportional_edit_falloff='SMOOTH', proportional_size=1)


def init_motor_position(Motor_type, Bottom_length, Motor_deflection):
    bpy.ops.object.select_all(action='DESELECT')
    for obj in bpy.data.objects:
        if ('0000' not in obj.name) and ('Camera' not in obj.name):
            obj.select = True
            obj.location = (0, 0, 0)
    bpy.ops.transform.rotate(value=1.5708, axis=(0, 1, 0), constraint_axis=(False, True, False),
                             # Rotate all the Motor part 90` at Y axis
                             constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                             proportional_edit_falloff='SMOOTH', proportional_size=1)
    bpy.ops.transform.rotate(value=-1.5708, axis=(1, 0, 0), constraint_axis=(True, False, False),
                             # Rotate all the Motor part 90` at x axis
                             constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                             proportional_edit_falloff='SMOOTH', proportional_size=1)
    bpy.ops.transform.rotate(value=3.14159, axis=(0, 0, 1), constraint_axis=(False, False, True),
                             constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                             proportional_edit_falloff='SMOOTH', proportional_size=1)

    if 'TypeB' in Motor_type:
        bpy.ops.transform.rotate(value=1.5708 + Motor_deflection[0], axis=(1, 0, 0),
                                 constraint_axis=(True, False, False),
                                 constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                 proportional_edit_falloff='SMOOTH', proportional_size=1)
        bpy.ops.transform.rotate(value=0 + Motor_deflection[1], axis=(0, 1, 0), constraint_axis=(False, True, False),
                                 constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                 proportional_edit_falloff='SMOOTH', proportional_size=1)
        bpy.ops.transform.rotate(value=0 + Motor_deflection[2], axis=(0, 0, 1), constraint_axis=(False, False, True),
                                 constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                 proportional_edit_falloff='SMOOTH', proportional_size=1)
        bpy.ops.transform.translate(value=(-0.01, 0, 0), constraint_axis=(False, False, False),
                                    constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                    proportional_edit_falloff='SMOOTH', proportional_size=1)
    if 'TypeA2' in Motor_type:
        bpy.ops.transform.rotate(value=1.5708 + Motor_deflection[0], axis=(1, 0, 0),
                                 constraint_axis=(True, False, False),
                                 constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                 proportional_edit_falloff='SMOOTH', proportional_size=1)
        bpy.ops.transform.rotate(value=0 + Motor_deflection[1], axis=(0, 1, 0), constraint_axis=(False, True, False),
                                 constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                 proportional_edit_falloff='SMOOTH', proportional_size=1)
        bpy.ops.transform.rotate(value=0 + Motor_deflection[2], axis=(0, 0, 1), constraint_axis=(False, False, True),
                                 constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                 proportional_edit_falloff='SMOOTH', proportional_size=1)
        bpy.ops.transform.translate(value=(-0.05, 0, -0.03), constraint_axis=(False, False, False),
                                    constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                    proportional_edit_falloff='SMOOTH', proportional_size=1)
    if 'TypeA2' not in Motor_type and 'TypeA' in Motor_type:
        bpy.ops.transform.rotate(value=0 + Motor_deflection[0], axis=(1, 0, 0),
                                 constraint_axis=(True, False, False),
                                 constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                 proportional_edit_falloff='SMOOTH', proportional_size=1)
        bpy.ops.transform.rotate(value=0 + Motor_deflection[1], axis=(0, 1, 0), constraint_axis=(False, True, False),
                                 constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                 proportional_edit_falloff='SMOOTH', proportional_size=1)
        bpy.ops.transform.rotate(value=0 + Motor_deflection[2], axis=(0, 0, 1), constraint_axis=(False, False, True),
                                 constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                 proportional_edit_falloff='SMOOTH', proportional_size=1)
        bpy.ops.transform.translate(value=(-0.02, 0.0, -0.02), constraint_axis=(False, False, False),
                                    constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                    proportional_edit_falloff='SMOOTH', proportional_size=1)
    bpy.ops.transform.translate(value=(random.uniform(-0.875, -0.9) + Bottom_length, 0.2, 1.14),
                                constraint_axis=(False, False, False),
                                constraint_orientation='GLOBAL', mirror=False, proportional='DISABLED',
                                proportional_edit_falloff='SMOOTH', proportional_size=1)


#############################cut the cuboids####################################
################################################################################
def get_coordinate_cam(cam_pos_x, cam_pos_y, cam_pos_z, alpha, beta, theta, cor):
    alpha = float(alpha)
    beta = float(beta)
    theta = float(theta)
    cor = np.array(cor).T
    cam_pos = np.array([float(cam_pos_x), float(cam_pos_y), float(cam_pos_z)]).T
    cor = cor - cam_pos

    c_mw = np.array([[math.cos(beta) * math.cos(theta), math.cos(beta) * math.sin(theta), -math.sin(beta)],
                     [-math.cos(alpha) * math.sin(theta) + math.sin(alpha) * math.sin(beta) * math.cos(theta),
                      math.cos(alpha) * math.cos(theta) + math.sin(alpha) * math.sin(beta) * math.sin(theta),
                      math.sin(alpha) * math.cos(beta)],
                     [math.sin(alpha) * math.sin(theta) + math.cos(alpha) * math.sin(beta) * math.cos(theta),
                      -math.sin(alpha) * math.cos(theta) + math.cos(alpha) * math.sin(beta) * math.sin(theta),
                      math.cos(alpha) * math.cos(beta)]])

    cor_new = c_mw.dot(cor)
    return tuple(cor_new)


def get_3d_bbox(scene_pc, cam_info, motor_info, motor_type, sequence_motor, bbox_path, save_bbox=True):
    # patch_scene = np.load(scene_npy)
    points = motor_points(scene_pc)
    points_in_blender = transfer_cam2obj(cam_info[0], cam_info[1], cam_info[2], cam_info[3],
                                         cam_info[4], cam_info[5], points)
    points_in_motor = rectify(motor_info[0], motor_info[1], motor_info[2], points_in_blender)
    pos_info, cor_box = findpoints(points_in_motor)
    if save_bbox is True:
        label_info = [motor_type + '_' + sequence_motor]
        pos_info = ['{:.6f}'.format(i) for i in pos_info]
        pos_info = list(map(str, pos_info))
        label_info.extend(pos_info)
        label_info.extend(motor_info)
        with open(bbox_path + '\\motor_3D_bounding_box.csv', 'a+', newline='') as f:
            csv_writer = csv.writer(f)
            csv_writer.writerow(label_info)
    deflected_motor = deflect(motor_info[0], motor_info[1], motor_info[2], cor_box)
    bbox = transfer_obj2cam(cam_info[0], cam_info[1], cam_info[2], cam_info[3],
                            cam_info[4], cam_info[5], deflected_motor)
    return bbox


def get_panel(point_1, point_2, point_3):
    x1 = point_1[0]
    y1 = point_1[1]
    z1 = point_1[2]

    x2 = point_2[0]
    y2 = point_2[1]
    z2 = point_2[2]

    x3 = point_3[0]
    y3 = point_3[1]
    z3 = point_3[2]

    a = (y2 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1)
    b = (z2 - z1) * (x3 - x1) - (z3 - z1) * (x2 - x1)
    c = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)
    d = 0 - (a * x1 + b * y1 + c * z1)

    return (a, b, c, d)


def read_CameraPosition(csv_path):
    camera_position = []
    with open(csv_path, "r+") as f:
        csv_read = csv.reader(f)
        for line in csv_read:
            camera_position.append(line[0:6])
    return camera_position


def read_MotorDeflection(csv_path):
    motor_deflection = []
    with open(csv_path, "r+") as f:
        csv_read = csv.reader(f)
        for line in csv_read:
            motor_deflection.append(line[6:9])
    return motor_deflection


def set_Boundingbox(panel_list, point_cor):
    if panel_list['panel_up'][0] * point_cor[0] + panel_list['panel_up'][1] * point_cor[1] + panel_list['panel_up'][2] * \
            point_cor[2] + panel_list['panel_up'][3] <= 0:  # panel 1
        if panel_list['panel_bot'][0] * point_cor[0] + panel_list['panel_bot'][1] * point_cor[1] + \
                panel_list['panel_bot'][2] * point_cor[2] + panel_list['panel_bot'][3] >= 0:  # panel 2
            if panel_list['panel_front'][0] * point_cor[0] + panel_list['panel_front'][1] * point_cor[1] + \
                    panel_list['panel_front'][2] * point_cor[2] + panel_list['panel_front'][3] <= 0:  # panel 3
                if panel_list['panel_behind'][0] * point_cor[0] + panel_list['panel_behind'][1] * point_cor[1] + \
                        panel_list['panel_behind'][2] * point_cor[2] + panel_list['panel_behind'][3] >= 0:  # panel 4
                    if panel_list['panel_right'][0] * point_cor[0] + panel_list['panel_right'][1] * point_cor[1] + \
                            panel_list['panel_right'][2] * point_cor[2] + panel_list['panel_right'][3] >= 0:  # panel 5
                        if panel_list['panel_left'][0] * point_cor[0] + panel_list['panel_left'][1] * point_cor[1] + \
                                panel_list['panel_left'][2] * point_cor[2] + panel_list['panel_left'][
                            3] >= 0:  # panel 6

                            return True
    return False


def add_noise(patch_motor):
    ######### add uniform noise ############
    for point in patch_motor:
        noise_x = random.uniform(-0.001, 0.001)
        noise_y = random.uniform(-0.001, 0.001)
        noise_z = random.uniform(-0.005, 0.005)
        point[2] += noise_x
        point[3] += noise_y
        point[4] += noise_z

    ############ add downsample ############
    return patch_motor


def change_intoPointNet(wholescene, noise):
    if noise:
        cor = wholescene[:, 5:8]
    else:
        cor = wholescene[:, 2:5]
    color = wholescene[:, 9:12]
    label = np.array([wholescene[:, 8]])
    new = np.concatenate((cor, color, label.T), axis=1)
    return new


def raw2scene(raw_data, noise=True):
    patch = []
    if noise is True:
        for point in raw_data:
            noise_x = random.uniform(-0.001, 0.001)
            noise_y = random.uniform(-0.001, 0.001)
            noise_z = random.uniform(-0.005, 0.005)
            point[5] += noise_x
            point[6] += noise_y
            point[7] += noise_z
            patch.append(point)
        patch = change_intoPointNet(np.array(patch), noise=noise)
    else:
        for point in raw_data:
            patch.append(point)
        patch = change_intoPointNet(np.array(patch), noise=False)
    return patch


def cut(data_to_cut, noise, camera_position_now):
    cly_bottom = random.uniform(0.66, 0.75)
    noise_xmin = random.uniform(-0.2, 0.1)
    noise_xmax = random.uniform(-0.2, 0.1)
    noise_ymin = random.uniform(-0.05, 0.1)
    noise_ymax = random.uniform(-0.1, 0.05)
    corners = [(-1.8 + noise_xmin, -0.2 + noise_ymin, 1.4), (-0.2 + noise_xmax, -0.2 + noise_ymin, 1.4),
               (-0.2 + noise_xmax, 0.65 + noise_ymax, 1.4), (-1.8 + noise_xmin, 0.65 + noise_ymax, 1.4),
               (-1.8 + noise_xmin, -0.2 + noise_ymin, cly_bottom), (-0.2 + noise_xmax, -0.2 + noise_ymin, cly_bottom),
               (-0.2 + noise_xmax, 0.65 + noise_ymax, cly_bottom), (-1.8 + noise_xmin, 0.65 + noise_ymax, cly_bottom)]
    cor_inCam = []
    for corner in corners:
        cor_cam_point = get_coordinate_cam(camera_position_now[0], camera_position_now[1], camera_position_now[2],
                                                camera_position_now[3], camera_position_now[4], camera_position_now[5],
                                                corner)
        cor_inCam.append(cor_cam_point)
    panel_1 = get_panel(cor_inCam[0], cor_inCam[1], cor_inCam[2])
    panel_2 = get_panel(cor_inCam[5], cor_inCam[6], cor_inCam[4])
    panel_3 = get_panel(cor_inCam[0], cor_inCam[3], cor_inCam[4])
    panel_4 = get_panel(cor_inCam[1], cor_inCam[2], cor_inCam[5])
    panel_5 = get_panel(cor_inCam[0], cor_inCam[1], cor_inCam[4])
    panel_6 = get_panel(cor_inCam[2], cor_inCam[3], cor_inCam[6])
    panel_list = {'panel_up': panel_1, 'panel_bot': panel_2, 'panel_front': panel_3, 'panel_behind': panel_4,
                  'panel_right': panel_5, 'panel_left': panel_6}

    patch_motor = []
    for point in data_to_cut:
        if not noise:
            point_cor = (point[2], point[3], point[4])
        else:
            point_cor = (point[5], point[6], point[7])
        if set_Boundingbox(panel_list, point_cor):
            patch_motor.append(point)
    if (noise):
        patch_motor = add_noise(patch_motor)

    patch_motor = change_intoPointNet(np.array(patch_motor), noise=noise)  # resort the file information N*13 -> N*7

    return patch_motor


def scan_cut(save_dir, motor_type, sequence_motor, cam_info, motor_info, save_scene=True, save_cuboid=True,
             scene_file_format='numpy', cuboid_file_format='numpy', bb_path=None, save_bbox=True):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.scene.camera = bpy.data.objects['Camera']
    bpy.data.objects['Camera'].select = True
    bpy.context.scene.objects.active = bpy.data.objects['Camera']
    sigma = random.uniform(0.001, 0.006)
    saved_numpy_path = save_dir + '\\' + 'Training_' + motor_type + '_' + sequence_motor + '_raw.numpy'
    scanner = bpy.data.objects['Camera']
    blensor.tof.scan_advanced(scanner, tof_res_x=1280, tof_res_y=960, evd_file=saved_numpy_path, noise_sigma=sigma)
    noiseNumpy_name = motor_type + '_' + sequence_motor + '_cuboid'
    motorNumpy_file = save_dir + '\\' + 'Training_' + motor_type + '_' + sequence_motor + '_raw00000' + '.numpy'
    sceneNumpy_name = motor_type + '_' + sequence_motor + '_scene'
    #########  transform the numpy file  #############
    motor_numpy = np.loadtxt(motorNumpy_file)
    filtered = cutNumpy(motor_numpy)
    filtered = ChangeLabel(filtered)
    filtered = Resort_IDX(filtered)
    if save_scene is True:
        filtered1 = raw2scene(raw_data=filtered, noise=True)
        np.save(save_dir + '\\' + sceneNumpy_name, filtered1)
        if save_bbox is True:
            scene_points = np.load(save_dir + '\\' + sceneNumpy_name + '.npy')
            bbox = get_3d_bbox(scene_points, cam_info, motor_info,
                               motor_type, sequence_motor, bbox_path=bb_path, save_bbox=True)
        if scene_file_format == 'both':
            scene_points = np.load(save_dir + '\\' + sceneNumpy_name + '.npy')
            points2pcd(save_dir + '\\' + sceneNumpy_name + '.pcd', scene_points)
        elif scene_file_format == 'pcd':
            scene_points = np.load(save_dir + '\\' + sceneNumpy_name + '.npy')
            points2pcd(save_dir + '\\' + sceneNumpy_name + '.pcd', scene_points)
            os.remove(save_dir + '\\' + sceneNumpy_name + '.npy')
    if save_cuboid is True:
        filtered2 = cut(data_to_cut=filtered, noise=True, camera_position_now=cam_info)
        np.save(save_dir + '\\' + noiseNumpy_name, filtered2)
        if cuboid_file_format == 'both':
            cuboid_pc = np.load(save_dir + '\\' + noiseNumpy_name + '.npy')
            points2pcd(save_dir + '\\' + noiseNumpy_name + '.pcd', cuboid_pc)
        elif cuboid_file_format == 'pcd':
            cuboid_pc = np.load(save_dir + '\\' + noiseNumpy_name + '.npy')
            points2pcd(save_dir + '\\' + noiseNumpy_name + '.pcd', cuboid_pc)
            os.remove(save_dir + '\\' + sceneNumpy_name + '.npy')
    os.remove(motorNumpy_file)
    bpy.ops.object.select_all(action='DESELECT')


def delete_motor():
    '''
        Delete all elements except Camera, lamp and clamping system
    '''
    filter_keep = ['Camera', '0000_Clamping_plc_enclosure', '0000_Clamping_foundation_left_clamp',
                   '0000_Clamping_countpart_right_clamp',
                   '0000_Clamping_Slider', '0000_Clamping_Pneumatic', '0000_Clamping_plate',
                   '0000_Clamping_operator_panel', '0000_Clamping_pillar', '0000_Clamping_Cylinder', '0000_Plane',
                   '0000_SurfPatch']
    for obj in bpy.data.objects:
        if obj.name not in filter_keep:
            bpy.data.objects.remove(obj)


def parse_opt():
    argv = sys.argv
    if "--" not in argv:
        argv = []
    else:
        argv = argv[argv.index("--") + 1:]  # get all args after "--"
    usage_text = (
            "Run Blender in background mode with this script:"
            "blender -b -P " + __file__ + " -- [options]"
    )
    parser = argparse.ArgumentParser(description=usage_text)
    parser.add_argument(
        '-i', '--input', dest='motor_path', metavar='FILE', help="path to the motor mesh models dir"
    )
    parser.add_argument(
        '-o', '--output', dest='save_path', metavar='FILE', help="path to the output dir"
    )
    parser.add_argument(
        '-clp', dest='clamping_path', metavar='FILE', default="E:\motor_dataset-master\clamping_system",
        help="path to the clamping system"
    )
    parser.add_argument(
        '-ss', dest='save_scene', action='store_false', help="whether to save the scene file"
    )
    parser.add_argument(
        '-sf', dest='scene_file_format', type=str, choices=['pcd', 'npy', 'both'], default='npy',
        help="set the scene file format"
    )
    parser.add_argument(
        '-bb', '--bbox', action='store_false', help="whether to save 3D bounding box of the motor in the scene file"
    )
    parser.add_argument(
        '-sc', dest='save_cuboid', action='store_false', help="whether to save the cuboid file"
    )
    parser.add_argument(
        '-cf', dest='cuboid_file_format', type=str, choices=['pcd', 'npy', 'both'], default='npy',
        help='set the cuboid file format'
    )
    # parser.add_argument(
    #     '-vs', dest='vis_scene', type=bool, default=True, help='visualize scene point cloud as image'
    # )
    # parser.add_argument(
    #     '-vc', dest='vis_cuboid', type=bool, default=True, help='visualize cuboid point cloud as image'
    # )
    # parser.add_argument(
    #     '-vb', dest='vis_bbox', type=bool, default=True, help='visualize 3D bounding box in scene image'
    # )
    parser.add_argument(
        '-ri', dest='rotation_from_image', action='store_false', help="random rotation of camera and motor from image"
    )
    parser.add_argument(
        '-cp', dest='csv_path', metavar='FILE', help="path to camera motor setting"
    )
    parser.add_argument(
        '-n', '--num', type=int, help='total generation number'
    )
    args = parser.parse_args(argv)
    if args.save_scene is False and args.save_cuboid is False:
        parser.error('No data will be output!')
    # if args.bbox != True and args.vb is True:
    #     parser.error('Bounding box info is NOT saved, please set -bb = True!')
    if (args.rotation_from_image is True) and (args.csv_path is None):
        parser.error("The csv path of image must be given!")
    if (args.rotation_from_image is False) and (args.csv_path is None):
        args.csv_path = args.save_path
    if args.num % 5 != 0:
        parser.error('Total generation number must be an integer multiple of 5!')
    return args


def main(args):
    global Cam_info_all, motor_deflection_all
    p = 3.1416 / 180
    filters = ["Motor.obj"]  ### the obj not to load
    file_base = args.motor_path  ###  directories to load motor
    list_type = os.listdir(file_base)  ### Path List for .obj file
    if 'motor_parameters.csv' in list_type:
        list_type.remove('motor_parameters.csv')
    list_type.sort()
    iteration = int(args.num / len(list_type))
    initial_clamp(args.clamping_path)  ### import clamping system
    for motor_type in list_type:
        save_path = args.save_path + '\\' + motor_type
        motor_file = file_base + '\\' + motor_type
        list_motor = os.listdir(motor_file)
        if 'data.csv' in list_motor:
            list_motor.remove('data.csv')
        list_motor.sort()
        ###################       get the needed data from data.csv     ############
        # sub_BottomLength_all = read_subBottomLength(file_path + '\\data.csv')
        bottomLength_all = read_bottomLength(motor_file + '\\data.csv')
        if args.rotation_from_image is True:
            Cam_info_all = read_CameraPosition(args.csv_path + '\\' + motor_type + '\\camera_motor_setting.csv')
            motor_deflection_all = read_MotorDeflection(
                args.csv_path + '\\' + motor_type + '\\camera_motor_setting.csv')
        else:
            create_csv(args.csv_path + '\\' + motor_type)
        if args.bbox is True:
            create_bbox_csv(args.save_path + '\\' + motor_type)
        ###################################################
        #   scan all the files from the holder            #
        ###################################################
        flag = 1
        for dirs in list_motor:
            if flag > iteration:
                break
            try:
                delete_motor()
            except KeyError:
                pass
            motor_path = motor_file + '\\' + dirs
            save_dir = save_path + '\\' + dirs
            k = dirs.split('_')
            bottomLength = bottomLength_all[int(k[1]) - 1]
            import_MotorPart_obj(motor_path, filters)
            if args.rotation_from_image is True:
                random_cam_info = list(map(float, Cam_info_all[int(k[1])]))
                motor_deflection = list(map(float, motor_deflection_all[int(k[1])]))
                random_cam_info = init_cam_position(random_cam_info)
                init_motor_position(motor_type, bottomLength, motor_deflection)
            else:
                random_cam_info = random_CameraPosition(radius_camera=random.uniform(2.8, 3.2))
                motor_deflection = np.random.uniform([-15 * p, -5 * p, -5 * p], [15 * p, 5 * p, 5 * p])
                init_motor_position(motor_type, bottomLength, motor_deflection)
                random_info = random_cam_info
                random_info.extend(motor_deflection)
                random_info = ['{:.6f}'.format(i) for i in random_info]
                csv_path = args.csv_path + '\\' + motor_type + '\\camera_motor_setting.csv'
                with open(csv_path, 'a+', newline='') as f:
                    csv_writer = csv.writer(f)
                    random_info = list(map(str, random_info))
                    csv_writer.writerow(random_info)

            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            scan_cut(save_dir=save_dir, motor_type=motor_type, sequence_motor=k[1], cam_info=random_cam_info,
                     motor_info=motor_deflection, save_scene=args.save_scene, save_cuboid=args.save_cuboid,
                     scene_file_format=args.scene_file_format, cuboid_file_format=args.cuboid_file_format,
                     bb_path=save_path, save_bbox=args.bbox)
            flag += 1


if __name__ == '__main__':
    opt = parse_opt()
    main(opt)
