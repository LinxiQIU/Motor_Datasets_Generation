import os
import numpy as np
import open3d as o3d
import csv
import math
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap



def vis_PointCloud(sampled, corner_box, bbox=True):
    #get only the koordinate from sampled
    sampled = np.asarray(sampled)
    PointCloud_koordinate = sampled[:, 0:3]
    label=sampled[:,6]
    labels = np.asarray(label)
    print(labels.shape)
    max_label = labels.max()
    cmap = ListedColormap(["navy", "darkgreen", "lime", "lavender", "yellow", "orange", "red"])
    colors = plt.get_cmap(cmap)(labels / (max_label if max_label>0 else 1))
    if bbox is True:    #visuell the point cloud and 3d bounding box
        lines = [[0, 1], [0, 2], [1, 3], [2, 3],
                 [4, 5], [4, 6], [5, 7], [6, 7],
                 [0, 4], [1, 5], [2, 6], [3, 7]]
        color = [[0, 1, 0] for i in range(len(lines))]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(corner_box)
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


def save_scene2img(patch_motor, corner_box, FileName=None, vis_bbox=True):
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
    if vis_bbox is True :
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
    vis.update_geometry(point_cloud)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(FileName)
    vis.destroy_window()

def main():
