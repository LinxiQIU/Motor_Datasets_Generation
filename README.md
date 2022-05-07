# Bosch-Motors-Dataset
This project is my master thesis and also a sub-project of the research project **AgiProbot** from KIT and Bosch. We develop a benchmark including 2D synthetic image datasets and 3D synthetic point cloud datasets, in which the key objects have multiple learn-able attributes with ground truth provided. In this project, small electric motors are used as the key objects. This part explains how to generate the datasets, including motor mesh model dataset, image dataset, point cloud dataset and noise point cloud dataset. We also build a synthetic **clamping** **system** with Blender to simulate the motor being clamped by the fixture in the real scenario.
## Software Preparation
* As our programming is based on the python language, we recommend a [Python>=3.7.0](https://www.python.org/) environment, including [Numpy](https://numpy.org/) and [Matplotlib](https://matplotlib.org/).
* We use [Blender 2.9](https://www.blender.org/) to generate the synthetic motor mesh model with the addon named motor factory. For specific details including how to install, you can see [Motor Factory](https://github.com/cold-soda-jay/blenderMotorFactoryVer2.0).
* The generation of image dataset is with the help of Blenderproc, a procedural Blender pipeline for photorealistic rendering. For Installation and tutorials, seen [Blenderproc](https://github.com/DLR-RM/BlenderProc). (I did this part of the work in an Ubuntu 20.04 environment)
* After generated the image dataset, we use another Blender addon named [Blensor](https://www.blensor.org/) to generate the point cloud dataset. Blensor is based on the Blender 2.79 version, may require additional libraries. (I used the third party released Blensor for Windows)
### 1. Motor Mesh Model Dataset 
We generate 5 kinds of synthetic motor mesh (Type A0, A1, A2, B0, B1) with Blender addon in randomly different size ranges in each motor's parts. 
> Run the Blender (version 2.9 above is recommended), open Text Editor(hotkey: Shift + F11) and open the script named `synthetic_motor_generate.py`. You should specify the path to save in BASE_DIR and the number of total motors in main() function. (Tips: The input here is the total number of motors. Since 5 types of motors will be generated, please ensure that the input is an integer multiple of 5) Click "Run Script".
```python
BASE_DIR = '/home/linxi/KIT/Thesis/Dataset/motor_mesh_model_50/' 
num_motor_for_dataset=50
```
### 2. Image Dataset Generation
In the image dataset, we merge motors and clamping system to generate 5 images for each scene, which are RGB image, distance image, normal image, semantic segmentated image for each part of the motor and a COCO-annotated image with a 2D bounding box of the motor, for the following tasks of the main project, we also add the 2D bounding box of the bolts. 
> The scripts here should be running by `blenderproc run script_name.py`. For example, you can generate one set of images for a motor by running the script `try.py`. You shoud set up the path of the clamping system in line 300, specify a motor type and number in line 310, line 311 and line 314 of the script.
![](https://github.com/LinxiQIU/Bosch-Motors-Dataset-generate/blob/main/try_setup.png)
```python
blenderproc run try.py 
```
> If you want to generate the dataset for the specified motor type, please run the script `batch_generation.py` and set the path of the motor type.
In each scene, we changed the position and euler rotation of the camera to increase the diversity of the scene. At the same time, we also randomly rotated the motor a little bit to simulate the improper placement of the motor in reality. This information will be saved in the csv file named camera_motor_setting.csv.
### 3. Point Cloud Dataset Generation
To generate the point cloud dataset, we are using [Blensor_1.0.18_RC_10_Windows](https://www.blensor.org/pages/downloads.html). Make sure it is installed correctly. Since I am using the Windows version of blensor, if you are using other systems, please pay attention to modifying the address format of the relevant python scripts. You can generate the whole point cloud dataset by running 'point_cloud_generation.py'
> First of all, copy the 'get_3d_bbox.py' and 'points2pcd.py' into the 'Blensor-1.0.18-Blender-2.79-Winx64/2.79/scripts/modules/'.
> Then open 'Command Prompt' in Windows, navigate to the Blensor directory and tpye in following command:
```python
blender -b -P path/of/point_cloud_generation.py -- -i path/of/input -o path/of/output -clp path/of/clamping_system -ss(save scene) -sf(scene file format) -bb(3d bounding box) -sc(save cuboid) -cf(cuboid file format) -roim(rotation from image dataset) -csvp path/of/csv -n(number of generation)
```

| cmd  | Description          | Type | Property |
| ------- | ----------------------------------------------------------| --- | ---------- |
| -b   | run Blender in background mode                        |       |            |
| -P   | python script                                          |      |            |
| -i   | path of motor mesh model                                | string     | obligatory |
| -o   | path of save directory                                  | string     | obligatory |
| -clp | path of clamping system                                 | string     | obligatory |
| -ss   | whether to save scene file (default=True)               | boolean    | optional   |
| -sf   | scene file format, option: npy, pcd, both (default: npy)  | string | optional |
| -bb   | whether to save 3D bounding box of motor (default=False)    | boolean |  optional  |
| -sc   | whether to save cuboid file (default=True)     | boolen | optional |
| -cf   | cuboid file format, option: npy, pcd, both (default: npy)  | string | optional |
| -ri | default=False: apply random rotation info and save. True: load rotation info from given csv file  | boolen  | optional |
| -cp | if -roim is False, save directory of rotation info.(default is save directory). if -roim is True, path of given csv file | string | optional/obligatory |
| -n    | number of total generation (an integer multiple of 5)     | integer | obligatory  |

> The point cloud dataset is composed of scene and cuboid point cloud, `-ss` and `-sc` default is True. If you enter `-sc`, it means sc=False, and the cuboid file will not be saved. We provide both numpy and pcd format, so 'both' should be entered after `-sf` and `-cf` respectively. We use the corresponding camera information saved in the camera_motor_setting.csv to scan the scene in point cloud to maintain correspondence with the image dataset, so `-roim` should set True and the path of csv file from the generated image dataset after `-csvp` must be given. You can also apply random rotation matrices and save it by default. `-n` represents the total number of point cloud files generated, since there are 5 motors in total, each motor will generate n/5 point cloud files. Here is the example command for my dataset.
```python
blender -b -P C:\Users\linux\PycharmProjects\Master\point_cloud_generation.py -- -i E:\motor_mesh_model -o E:\point_cloud_dataset -sf both -cf both -bb -roim -csvp E:\image_dataset_50 -n 50
```

> We mark the position of the motor in the point cloud scene with a 3D bounding box, and save the three-dimensional coordinates of the center of each motor and the length, width and height of the entire motor in motor_3D_bounding_box.csv for the deep learning task of 3D object detection by running `vis_point_cloud.py`.

![](https://github.com/LinxiQIU/Motor_Datasets_Generation/blob/master/images/scene_img.jpg)
> We provide each motor with both scene and cuboid point cloud in Numpy and PCD format. You can convert the generated Numpy file to PCD by running `points2pcd.py`, if you only generate the default numpy files at the beginning.
### 4. Point Cloud Dataset augmentation
On the basis of the point cloud dataset in the previous step, we add more random noises to augment data. For example, we add a cover randomly above the motor, randomly move the clamping parts. Here is a sample image for augmented point cloud of cuboid.  
![](https://github.com/LinxiQIU/Motor_Datasets_Generation/blob/master/images/cuboid_img.jpg)
You can get the whole augmented cuboid point cloud dataset by running `augmented_pc_generation.py` with Blensor. 
> Copy the 'get_3d_bbox.py' and 'points2pcd.py' into the 'Blensor/2.79/scripts/modules/'
> Open 'Command Prompt' in Windows, navigate to the Blensor directory and type in following command:
```python
blender -b -P path/of/augmented_pc_generation.py -- -i path/of/input -o path/of/output -clp path/of/clamping_system -ss(save scene) -sf(scene file format) -bb(3d bounding box) -sc(save cuboid) -cf(cuboid file format) -roim(rotation from image dataset) -csvp path/of/csv -n(number of generation)
```
Here is the example command for the demo.
![](https://github.com/LinxiQIU/Motor_Datasets_Generation/blob/master/images/blensor_cmd.png)
