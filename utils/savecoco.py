#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 18 14:52:34 2022

@author: linxi
"""

from PIL import Image, ImageFont, ImageDraw
from tqdm import tqdm
import os
import numpy as np
import json



def save_coco(coco_path):
    with open(os.path.join(coco_path, 'coco_data/coco_annotations.json')) as f:
        annotations = json.load(f)
        categories = annotations['categories']
        images = annotations['images']
        annotations = annotations['annotations']

    im = Image.open(os.path.join(coco_path+'/coco_data', images[0]['file_name']))

    def get_category(_id):
        category = [category["name"] for category in categories if category["id"] == _id]
        return str(category[0])

    def rle_to_binary_mask(rle):
        binary_array = np.zeros(np.prod(rle.get('size')), dtype=np.bool)
        counts = rle.get('counts')

        start = 0
        for i in range(len(counts)-1):
            start += counts[i]
            end = start + counts[i+1]
            binary_array[start:end] = (i + 1) % 2

        binary_mask = binary_array.reshape(*rle.get('size'), order='F')

        return binary_mask

    font = ImageFont.load_default()
    # Add bounding boxes and masks
    for idx, annotation in enumerate(annotations):
        if annotation["image_id"] == 0:
            draw = ImageDraw.Draw(im)
            bb = annotation['bbox']
            draw.rectangle(((bb[0], bb[1]), (bb[0] + bb[2], bb[1] + bb[3])), fill=None, outline="red")
            draw.text((bb[0] + 2, bb[1] + 2), get_category(annotation["category_id"]), font=font)
            if isinstance(annotation["segmentation"], dict):
                im.putalpha(255)
                rle_seg = annotation["segmentation"]
                item = rle_to_binary_mask(rle_seg).astype(np.uint8) * 255
                item = Image.fromarray(item, mode='L')
                overlay = Image.new('RGBA', im.size)
                draw_ov = ImageDraw.Draw(overlay)
                rand_color = np.random.randint(0,256,3)
                draw_ov.bitmap((0, 0), item, fill=(rand_color[0], rand_color[1], rand_color[2], 128))
                im = Image.alpha_composite(im, overlay)
            else:
                # go through all polygons and plot them
                for item in annotation['segmentation']:
                    poly = Image.new('RGBA', im.size)
                    draw = ImageDraw.Draw(poly)
                    rand_color = np.random.randint(0,256,3)
                    pdraw.polygon(item, fill=(rand_color[0], rand_color[1], rand_color[2], 127), outline=(255, 255, 255, 255))
                    im.paste(poly, mask=poly)
    
    im.save(os.path.join(output_folder, 'coco_annotated_0.png'), "PNG")
    # im.show()

if __name__ == "__main__":
    motor_dir = '/home/linxi/KIT/Thesis/Dataset/images_dataset/TypeB1'
    motor_ls = os.listdir(motor_dir)
    if 'camera_motor_setting.csv' in motor_ls:
        motor_ls.remove('camera_motor_setting.csv')
    motor_ls.sort()
    for dirs in tqdm(motor_ls, total=len(motor_ls)):
        output_folder = os.path.join(motor_dir, dirs)
        save_coco(output_folder)