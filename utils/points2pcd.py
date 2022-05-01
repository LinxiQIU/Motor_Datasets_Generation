import os
import numpy as np


def points2pcd(pcd_file_path, points):
    handle = open(pcd_file_path, 'a')

    point_num = points.shape[0]
    if point_num == 1228800:
        handle.write(
            '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb label\nSIZE 4 4 4 4 4\nTYPE F F F U U\nCOUNT 1 1 1 1 1')
        string = '\nWIDTH ' + str(1280)
        handle.write(string)
        handle.write('\nHEIGHT 960\nVIEWPOINT 0 0 0 1 0 0 0')
        string = '\nPOINTS ' + str(point_num)
        handle.write(string)
        handle.write('\nDATA ascii')

        # int rgb = ((int)r << 16 | (int)g << 8 | (int)b);
        for i in range(point_num):
            r, g, b = points[i, 3], points[i, 4], points[i, 5]
            rgb = int(r) << 16 | int(g) << 8 | int(b)
            string = '\n' + str(points[i, 0]) + ' ' + str(points[i, 1]) + ' ' + str(points[i, 2]) + ' ' + str(
                rgb) + ' ' + str(points[i, 6])
            handle.write(string)
        handle.close()
    else:
        handle.write(
            '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb label\nSIZE 4 4 4 4 4\nTYPE F F F U U\nCOUNT 1 1 1 1 1')
        string = '\nWIDTH ' + str(point_num)
        handle.write(string)
        handle.write('\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0')
        string = '\nPOINTS ' + str(point_num)
        handle.write(string)
        handle.write('\nDATA ascii')

        # int rgb = ((int)r << 16 | (int)g << 8 | (int)b);
        for i in range(point_num):
            r, g, b = points[i, 3], points[i, 4], points[i, 5]
            rgb = int(r) << 16 | int(g) << 8 | int(b)
            string = '\n' + str(points[i, 0]) + ' ' + str(points[i, 1]) + ' ' + str(points[i, 2]) + ' ' + str(
                rgb) + ' ' + str(points[i, 6])
            handle.write(string)
        handle.close()




def main():
    base_dir = 'E:\\noise_point50\TypeB1'
    root, Motortype = os.path.split(base_dir)
    List_motor = os.listdir(base_dir)
    if 'camera_motor_setting.csv' in List_motor:
        List_motor.remove('camera_motor_setting.csv')
    if 'motor_3D_bounding_box.csv' in List_motor:
        List_motor.remove('motor_3D_bounding_box.csv')
    List_motor.sort()
    for dirs in List_motor:
        k = dirs.split('_')
        scene_npy = base_dir + '\\' + dirs + '\\' + Motortype + '_' + k[1] + "_scene.npy"
        scene_pcd = base_dir + '\\' + dirs + '\\' + Motortype + '_' + k[1] + "_scene.pcd"
    # f = 'E:\Result\pcd\TypeA2\Training_TypeA2_0001_scene.npy'
        scene_points = np.load(scene_npy)
        points2pcd(scene_pcd, scene_points)

        cuboid_npy = base_dir + '\\' + dirs + '\\' + Motortype + '_' + k[1] + "_cuboid.npy"
        cuboid_pcd = base_dir + '\\' + dirs + '\\' + Motortype + '_' + k[1] + "_cuboid.pcd"
    # pcd_file_path = 'E:\Result\pcd\TypeA2\Training_TypeA2_0001_scene.npy.pcd'
        cuboid_points = np.load(cuboid_npy)
        points2pcd(cuboid_pcd, cuboid_points)
    # print(points[1])



if __name__ == '__main__':
    main()