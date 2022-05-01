import os
import numpy as np
import csv
import math


def findpoints(points, save=False):
    # point_num = points.shape[1]
    # # print(point_num)
    x_max = points[0].max()
    x_min = points[0].min()
    y_max = points[1].max()
    y_min = points[1].min()
    z_max = points[2].max()
    z_min = points[2].min()
    x = (x_max - x_min)/2 + x_min
    y = (y_max - y_min)/2 + y_min
    z = (z_max - z_min)/2 + z_min
    h = z_max - z_min
    w = y_max - y_min
    l = x_max - x_min
    corner_box = np.array([[x-l/2, y-w/2, z-h/2],[x+l/2, y-w/2, z-h/2],[x-l/2, y+w/2, z-h/2],[x+l/2, y+w/2, z-h/2],
                           [x-l/2, y-w/2, z+h/2],[x+l/2, y-w/2, z+h/2],[x-l/2, y+w/2, z+h/2],[x+l/2, y+w/2, z+h/2]])
    # corner_box = np.array([[x_min, y_min, z_min],[x_max, y_min, z_min],[x_min, y_max, z_min],[x_max, y_max, z_min],
    #                        [x_min, y_min, z_max],[x_max, y_min, z_max],[x_min, y_max, z_max],[x_max, y_max, z_max]])

    return [x, y, z, h, w, l], corner_box

def motor_points(x):
    y = x[x[:, 6] != 0.0]
    z = y[:, :3]
    return z


def read_Cam_Pos(csv_path):
    cam_pos = []
    with open(csv_path, "r+") as f:
        csv_read = csv.reader(f)
        for line in csv_read:
            cam_pos.append(line[:6])
    return cam_pos


def read_motor_deflection(csv_path):
    motor_deflection = []
    with open(csv_path, "r+") as f:
        csv_read = csv.reader(f)
        for line in csv_read:
            motor_deflection.append(line[6:9])
    return motor_deflection


def rotation_matrix(alpha, beta, theta):
    M = np.array([[math.cos(theta)*math.cos(beta), -math.sin(theta)*math.cos(alpha)+math.cos(theta)*math.sin(beta)*math.sin(alpha), math.sin(theta)*math.sin(alpha)+math.cos(theta)*math.sin(beta)*math.cos(alpha)],
                  [math.sin(theta)*math.cos(beta), math.cos(theta)*math.cos(alpha)+math.sin(theta)*math.sin(beta)*math.sin(alpha), -math.cos(theta)*math.sin(alpha)+math.sin(theta)*math.sin(beta)*math.cos(alpha)],
                 [-math.sin(beta), math.cos(beta)*math.sin(alpha), math.cos(beta)*math.cos(alpha)]])
    return M


def transfer_cam2obj(cam_pos_x, cam_pos_y, cam_pos_z, alpha, beta, theta, points):
    alpha = float(alpha)
    beta = float(beta)
    theta = float(theta)
    cam = np.array([float(cam_pos_x), float(cam_pos_y), float(cam_pos_z)])
    cam_pos = np.full((points.shape[0], 3), cam)
    c_mw = np.array([[math.cos(beta) * math.cos(theta), math.cos(beta) * math.sin(theta), -math.sin(beta)],
                     [-math.cos(alpha) * math.sin(theta) + math.sin(alpha) * math.sin(beta) * math.cos(theta),
                      math.cos(alpha) * math.cos(theta) + math.sin(alpha) * math.sin(beta) * math.sin(theta),
                      math.sin(alpha) * math.cos(beta)],
                     [math.sin(alpha) * math.sin(theta) + math.cos(alpha) * math.sin(beta) * math.cos(theta),
                      -math.sin(alpha) * math.cos(theta) + math.cos(alpha) * math.sin(beta) * math.sin(theta),
                      math.cos(alpha) * math.cos(beta)]])
    points_in_blender = np.linalg.inv(c_mw).dot(points.T) + cam_pos.T
    return points_in_blender


def transfer_obj2cam(cam_pos_x, cam_pos_y, cam_pos_z, alpha, beta, theta, points):
    alpha = float(alpha)
    beta = float(beta)
    theta = float(theta)
    cam = (float(cam_pos_x), float(cam_pos_y), float(cam_pos_z))
    cam_pos = np.full((points.shape[1], 3), cam)
    points = points - cam_pos.T
    # M = rotation_matrix(alpha, beta, theta)
    c_mw = np.array([[math.cos(beta) * math.cos(theta), math.cos(beta) * math.sin(theta), -math.sin(beta)],
                     [-math.cos(alpha) * math.sin(theta) + math.sin(alpha) * math.sin(beta) * math.cos(theta),
                      math.cos(alpha) * math.cos(theta) + math.sin(alpha) * math.sin(beta) * math.sin(theta),
                      math.sin(alpha) * math.cos(beta)],
                     [math.sin(alpha) * math.sin(theta) + math.cos(alpha) * math.sin(beta) * math.cos(theta),
                      -math.sin(alpha) * math.cos(theta) + math.cos(alpha) * math.sin(beta) * math.sin(theta),
                      math.cos(alpha) * math.cos(beta)]])
    cor_new = c_mw.dot(points)
    return cor_new


def deflect(alpha, beta, theta, points):
    alpha = float(alpha)
    beta = float(beta)
    theta = float(theta)
    M = rotation_matrix(alpha, beta, theta)
    points_motor = M.dot(points.T)
    return points_motor


def rectify(alpha, beta, theta, points):
    alpha = float(alpha)
    beta = float(beta)
    theta = float(theta)
    M = rotation_matrix(alpha, beta, theta)
    points_motor = np.linalg.inv(M).dot(points)
    return points_motor


def create_bbox_csv(save_dir):
    if not os.path.exists(save_dir):
        os.mkdir(save_dir)
    save_dir = save_dir + '\\motor_3D_bounding_box.csv'
    with open(save_dir, 'a+', newline='') as f:
        csv_writer = csv.writer(f)
        head = ["motor_id", "location_x", "location_x", "location_z", "height", "width", "length", "eulerX", "eulerY", "eulerZ"]
        csv_writer.writerow(head)


def main():
    base_dir = 'E:\\TypeA1'
    List_motor = os.listdir(base_dir)
    if 'camera_motor_setting.csv' in List_motor:
        List_motor.remove('camera_motor_setting.csv')
    if 'motor_3D_bounding_box.csv' in List_motor:
        List_motor.remove('motor_3D_bounding_box.csv')
    List_motor.sort()

    root, motortype = os.path.split(base_dir)
    csv_path = base_dir + '\\camera_motor_setting.csv'
    Cam_info_all = read_Cam_Pos(csv_path)
    Motor_deflection_all = read_motor_deflection(csv_path)
    create_bbox_csv(base_dir)
    for dirs in List_motor:
        k = dirs.split('_')
        scene_npy = base_dir + '\\' + dirs + '\\' + motortype + '_' + k[1] + '_scene.npy'
        patch_motor = np.load(scene_npy)
        points = motor_points(patch_motor)
        Cam_info = Cam_info_all[int(k[1])]
        Motor_deflection = Motor_deflection_all[int(k[1])]
        points_in_blender = transfer_cam2obj(Cam_info[0], Cam_info[1], Cam_info[2], Cam_info[3], Cam_info[4], Cam_info[5], points)
    # print(np.shape(points_in_blender))
        points_in_motor = rectify(Motor_deflection[0], Motor_deflection[1], Motor_deflection[2], points_in_blender)
    # print(np.shape(points_in_motor))
        pos_info, cor_box = findpoints(points_in_motor)
        root, scene_name = os.path.split(scene_npy)
        label_info = [scene_name]
        pos_info = ['{:.6f}'.format(i) for i in pos_info]
        pos_info = list(map(str, pos_info))
        label_info.extend(pos_info)
        label_info.extend(Motor_deflection)
        with open(base_dir + '\\motor_3D_bounding_box.csv', 'a+', newline='') as f:
            csv_writer = csv.writer(f)
            csv_writer.writerow(label_info)

        deflected_motor = deflect(Motor_deflection[0], Motor_deflection[1], Motor_deflection[2], cor_box)
        cor_box_new = transfer_obj2cam(Cam_info[0], Cam_info[1], Cam_info[2], Cam_info[3], Cam_info[4], Cam_info[5], deflected_motor)

        # Visuell_PointCloud(patch_motor, cor_box_new.T)
        # save_scene2img(patch_motor, cor_box_new.T, FileName=base_dir + '\\' + dirs + '\\' + "scene.jpg", vis_bbox=True)
        # cuboid_npy = base_dir + '\\' + dirs + '\\' + motortype + '_' + k[1] + '_cuboid.npy'
        # cuboid = np.load(cuboid_npy)
        # save_cuboid2img(cuboid, FileName=base_dir + '\\' + dirs + '\\' + "cuboid.jpg")

if __name__ == '__main__':
    main()