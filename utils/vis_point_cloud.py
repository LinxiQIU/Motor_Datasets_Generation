import os
import numpy as np
import open3d as o3d
import csv
import math
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


def vis_PointCloud(sampled, corner_box=None):
    #get only the koordinate from sampled
    sampled = np.asarray(sampled)
    PointCloud_koordinate = sampled[:, 0:3]
    label=sampled[:,6]
    labels = np.asarray(label)
    print(labels.shape)
    max_label = label.max()
    cmap = ListedColormap(["navy", "darkgreen", "lime", "lavender", "yellow", "orange", "red", "pink"])
    colors = plt.get_cmap(cmap)(label / (max_label + 1))
    if corner_box is not None:    #visuell the point cloud and 3d bounding box
        lines = [[0, 1], [0, 2], [1, 3], [2, 3],
                 [4, 5], [4, 6], [5, 7], [6, 7],
                 [0, 4], [1, 5], [2, 6], [3, 7]]
        color = [[0, 1, 0] for i in range(len(lines))]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(corner_box.T)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(color)
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(PointCloud_koordinate)
        point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw_geometries([point_cloud, line_set])
    else:
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(PointCloud_koordinate)
        point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw_geometries([point_cloud])


def save_scene2img(patch_motor, corner_box=None, FileName=None):
    sampled = np.asarray(patch_motor)
    PointCloud_koordinate = sampled[:, 0:3]
    label=sampled[:,6]
    labels = np.asarray(label)
    print(labels.shape)
    max_label = label.max()
    cmap = ListedColormap(["navy", "darkgreen", "lime", "lavender", "yellow", "orange", "red", "pink"])
    colors = plt.get_cmap(cmap)(label / (max_label + 1))
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(PointCloud_koordinate)
    point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1280, height=960)
    vis.add_geometry(point_cloud)
    if corner_box is not None:
        lines = [[0, 1], [0, 2], [1, 3], [2, 3],
                 [4, 5], [4, 6], [5, 7], [6, 7],
                 [0, 4], [1, 5], [2, 6], [3, 7]]
        color = [[0, 1, 0] for i in range(len(lines))]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(corner_box)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(color)
        vis.add_geometry(line_set)
    vis.get_render_option().point_size = 1.0
    ctr = vis.get_view_control()
    ctr.set_zoom(0.4)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(FileName)
    vis.destroy_window()


def save_cuboid2img(patch_motor, FileName=None):
    sampled = np.asarray(patch_motor)
    PointCloud_koordinate = sampled[:, 0:3]
    label=sampled[:,6]
    labels = np.asarray(label)
    print(labels.shape)
    max_label = labels.max()
    cmap = ListedColormap(["navy", "darkgreen", "lime", "lavender", "yellow", "orange", "red"])
    colors = plt.get_cmap(cmap)(labels / (max_label if max_label>0 else 1))
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(PointCloud_koordinate)
    point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1280, height=960)
    vis.add_geometry(point_cloud)
    vis.get_render_option().point_size = 1.0
    ctr = vis.get_view_control()
    ctr.set_zoom(0.8)
    vis.update_geometry(point_cloud)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(FileName)
    vis.destroy_window()


def read_cam_motor(csv_path):
    cam_pos = []
    motor_deflection = []
    with open(csv_path + '\\camera_motor_setting.csv', "r+") as f:
        csv_read = csv.reader(f)
        for line in csv_read:
            cam_pos.append(line[:6])
            motor_deflection.append(line[6:9])
    return cam_pos, motor_deflection



def rotation_matrix(alpha, beta, theta):
    M = np.array([[math.cos(theta)*math.cos(beta), -math.sin(theta)*math.cos(alpha)+math.cos(theta)*math.sin(beta)*math.sin(alpha),
                   math.sin(theta)*math.sin(alpha)+math.cos(theta)*math.sin(beta)*math.cos(alpha)],
                  [math.sin(theta)*math.cos(beta), math.cos(theta)*math.cos(alpha)+math.sin(theta)*math.sin(beta)*math.sin(alpha),
                   -math.cos(theta)*math.sin(alpha)+math.sin(theta)*math.sin(beta)*math.cos(alpha)],
                  [-math.sin(beta), math.cos(beta)*math.sin(alpha), math.cos(beta)*math.cos(alpha)]])
    return M


def deflect(alpha, beta, theta, points):
    alpha = float(alpha)
    beta = float(beta)
    theta = float(theta)
    M = rotation_matrix(alpha, beta, theta)
    points_motor = M.dot(points.T)
    return points_motor


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


def get_bbox(bbox_csv, cam_motor_csv, k):
    bbox = []
    with open(bbox_csv + '\\motor_3D_bounding_box.csv', "r+") as f:
        csv_read = csv.reader(f)
        for line in csv_read:
            bbox.append(line[1:10])
    x = float(bbox[k][0])
    y = float(bbox[k][1])
    z = float(bbox[k][2])
    h = float(bbox[k][3])
    w = float(bbox[k][4])
    l = float(bbox[k][5])
    cor_box = np.array([[x - l / 2, y - w / 2, z - h / 2], [x + l / 2, y - w / 2, z - h / 2],
                           [x - l / 2, y + w / 2, z - h / 2], [x + l / 2, y + w / 2, z - h / 2],
                           [x - l / 2, y - w / 2, z + h / 2], [x + l / 2, y - w / 2, z + h / 2],
                           [x - l / 2, y + w / 2, z + h / 2], [x + l / 2, y + w / 2, z + h / 2]])
    cam_info_all, motor_deflection_all = read_cam_motor(cam_motor_csv)
    cam_info = cam_info_all[k+1]
    motor_def = motor_deflection_all[k+1]
    deflected_motor = deflect(motor_def[0], motor_def[1], motor_def[2], cor_box)
    corner_box = transfer_obj2cam(cam_info[0], cam_info[1], cam_info[2], cam_info[3], cam_info[4], cam_info[5], deflected_motor)
    return corner_box


