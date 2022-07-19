#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 14 20:11:09 2022

@author: linxi
"""


import os
import json
import shutil


def read_bbox(file, coco_file, idx=None):
    with open(file + '/coco_data/coco_annotations.json') as f:
        d = json.load(f)
        n = len(d['annotations'])
        for i in range(n):
            category_id = d['annotations'][i]['category_id']
            if category_id == 5:
                label = [str(0)]
            elif category_id == 6:
                label = [str(1)]
            else:
                label = [str(2)]
            bbox = d['annotations'][i]['bbox']
            norm_bbox = [(bbox[0] + bbox[2]/2)/1280, (bbox[1] + bbox[3]/2)/960, bbox[2]/1280, bbox[3]/960]
            norm_bbox = ['{:.6f}'.format(i) for i in norm_bbox]
            label.extend(norm_bbox)
            # dst_label = os.path.join(coco_dataset, 'labels')
            txt_path = os.path.join(coco_file, 'labels')
            if not os.path.exists(txt_path):
                os.makedirs(txt_path)
            with open(txt_path + '/im' + str(idx) + '.txt', 'a+') as fp:
                for item in label:
                    fp.write('%s '%item)
                fp.write('\n')


def copy_img(raw_img, coco_file, idx=None):
    dst_path = os.path.join(coco_file, 'images')
    if not os.path.exists(dst_path):
        os.mkdir(dst_path)
    dst_img = os.path.join(dst_path, 'im' + str(idx) + '.jpg')    
    shutil.copyfile(raw_img, dst_img)


def main():
    raw_file = '/home/linxi/KIT/Thesis/Dataset/images_dataset'
    coco_file = '/home/linxi/KIT/Thesis/Dataset/motor_coco825'
    list_types_motor = os.listdir(raw_file)
    if 'scene_setting.csv' in list_types_motor:
        list_types_motor.remove('scene_setting.csv')
    list_types_motor.sort()
    flag = 0
    for motor_type in list_types_motor:
        file = raw_file + '/' + motor_type
        list_motor = os.listdir(file)
        if 'camera_motor_setting.csv' in list_motor:
            list_motor.remove('camera_motor_setting.csv')
        list_motor.sort()
        for dirs in list_motor:
            raw_img = file + '/' + dirs + '/coco_data/images/000000.jpg'
            copy_img(raw_img, coco_file, flag)
            read_bbox(file + '/' + dirs, coco_file, flag)
            flag += 1


if __name__ == '__main__':
    main()