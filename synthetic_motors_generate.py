import os
import bpy
import numpy as np
import random
import math
'''
    produce motor obj to generate the synthetic point cloud dataset for next step
    here i have generate 1250 motors (include 250 motors for each type)
    all this part was generated on the Mac os system
'''


'''
Value-distribution:
    Type A_Type 1 :
                    Bottom Length = 6.2 - 8.0  step = 0.03
                    Sub Bottom Length = 0.6 - 2  setp = 0.03
                    Lower Gear Dia = 3.5 - 4.5  step = 0.03
                    Lower Gear Position = 3.6 - 4.2
                    Upper Gear Dia = 5 - 6.5
                    Bolts Type = 'mf_Bit_Slot', 'mf_Bit_Torx', 'mf_Bit_Corss',
                    Bolt Orientation = 'mf_all_same', 'mf_all_random'
                    Position of Bolt 1 on lower gear = 190 - 230    step = 5
                    Position of Bolts on lower gear = 320 - 350    step = 5
                    Random of Position around lower gear = True/False
                    Number of Bolts = 1, 2, 3
                    Position of bolt 1 on larger gear = 0 - 105    step = 1
                    Position of bolt 2 on larger gear = 106 - 210    step = 1
The rule I used now:
                    Each changed parameter in changed value + the other parameter in default values  ->  3 groups with different type of bolts
'''
def create_Motor_Obj_A(Extension_Type_A, Bolt_type_mode, changed_parameter, Upper_Bolt_Nummber, Bottom_Corrosion_Type, Bolt_Corrosion_Type, save_dir) :         # Motor_type = ['Type_A', 'Type_B']  Type_A_Extension = ['Type_1', 'Type_2']
   
    try :
        if bpy.data.objects['Cube'] :       # Delete the Cube object and Lamp object from Blender's intialization
            bpy.data.objects.remove(bpy.data.objects['Cube'])
        if bpy.data.objects['Light'] :
            bpy.data.objects.remove(bpy.data.objects['Light'])
    except KeyError :
        pass

    try :
        if bpy.data.objects['Motor']:
            bpy.data.objects.remove(bpy.data.objects['Motor'])
    except KeyError:
        pass
    
    Bolts_Type = ['mf_Bit_Slot', 'mf_Bit_Torx', 'mf_Bit_Corss']
    if Extension_Type_A == 'mf_Extension_Type_1' :
        if Bolt_type_mode == 'Torx' :
            bpy.ops.mesh.add_motor(change=True, mf_Color_Render=True, mf_Lower_Gear_Bolt_Random=True,  mf_Upper_Gear_Bolt_Random=True, temp_save=True, mf_Top_Type='mf_Top_Type_A',
                    mf_Extension_Type_A='mf_Extension_Type_1', mf_Bottom_Length=changed_parameter['changed_BL'], mf_Sub_Bottom_Length=changed_parameter['changed_SBL'], mf_Lower_Gear_Dia=changed_parameter['changed_LGD'],
                    mf_Lower_Gear_Position=changed_parameter['changed_LGP'], mf_Upper_Gear_Dia=changed_parameter['changed_UGD'], mf_Upper_Bolt_Nummber=Upper_Bolt_Nummber, 
                    mf_Bolt_Orientation='mf_all_random', mf_Bit_Type='mf_Bit_Torx', mf_corrosion_Type_Bolt=Bolt_Corrosion_Type, mf_corrosion_Percent_Bolt=changed_parameter['bolt_corrosion_percent'],
                    mf_corrosion_Type_Bottom=Bottom_Corrosion_Type, mf_corrosion_Percent_Bottom=changed_parameter['bottom_corrosion_percent'], save_path=save_dir)
            bpy.data.objects.remove(bpy.data.objects['Motor'])
        elif Bolt_type_mode == 'Random' :
            bpy.ops.mesh.add_motor(change=True, mf_Color_Render=True, mf_Lower_Gear_Bolt_Random=True,  mf_Upper_Gear_Bolt_Random=True, temp_save=True, mf_Top_Type='mf_Top_Type_A',
                    mf_Extension_Type_A='mf_Extension_Type_1', mf_Bottom_Length=changed_parameter['changed_BL'], mf_Sub_Bottom_Length=changed_parameter['changed_SBL'], mf_Lower_Gear_Dia=changed_parameter['changed_LGD'],
                    mf_Lower_Gear_Position=changed_parameter['changed_LGP'], mf_Upper_Gear_Dia=changed_parameter['changed_UGD'], mf_Upper_Bolt_Nummber=Upper_Bolt_Nummber, 
                    mf_Bolt_Orientation='mf_all_random', mf_Bit_Type=random.choice(Bolts_Type), mf_corrosion_Type_Bolt=Bolt_Corrosion_Type, mf_corrosion_Percent_Bolt=changed_parameter['bolt_corrosion_percent'],
                    mf_corrosion_Type_Bottom=Bottom_Corrosion_Type, mf_corrosion_Percent_Bottom=changed_parameter['bottom_corrosion_percent'], save_path=save_dir)
            bpy.data.objects.remove(bpy.data.objects['Motor'])

    elif Extension_Type_A == 'mf_Extension_Type_2' :
        if Bolt_type_mode == 'Torx' :
            bpy.ops.mesh.add_motor(change=True, mf_Color_Render=True, mf_Lower_Gear_Bolt_Random=True,  mf_Upper_Gear_Bolt_Random=True, temp_save=True, mf_Top_Type='mf_Top_Type_A',
                    mf_Extension_Type_A='mf_Extension_Type_2', mf_Bottom_Length=changed_parameter['changed_BL'], mf_Sub_Bottom_Length=changed_parameter['changed_SBL'], mf_Lower_Gear_Dia=changed_parameter['changed_LGD'],
                    mf_Lower_Gear_Position=changed_parameter['changed_LGP'], mf_Upper_Gear_Dia=changed_parameter['changed_UGD'], mf_Upper_Bolt_Nummber=Upper_Bolt_Nummber, 
                    mf_Bolt_Orientation='mf_all_random', mf_Bit_Type='mf_Bit_Torx', mf_corrosion_Type_Bolt=Bolt_Corrosion_Type, mf_corrosion_Percent_Bolt=changed_parameter['bolt_corrosion_percent'],
                    mf_corrosion_Type_Bottom=Bottom_Corrosion_Type, mf_corrosion_Percent_Bottom=changed_parameter['bottom_corrosion_percent'], save_path=save_dir)
            bpy.data.objects.remove(bpy.data.objects['Motor'])
        elif Bolt_type_mode == 'Random' :
            bpy.ops.mesh.add_motor(change=True, mf_Color_Render=True, mf_Lower_Gear_Bolt_Random=True,  mf_Upper_Gear_Bolt_Random=True, temp_save=True, mf_Top_Type='mf_Top_Type_A',
                    mf_Extension_Type_A='mf_Extension_Type_2', mf_Bottom_Length=changed_parameter['changed_BL'], mf_Sub_Bottom_Length=changed_parameter['changed_SBL'], mf_Lower_Gear_Dia=changed_parameter['changed_LGD'],
                    mf_Lower_Gear_Position=changed_parameter['changed_LGP'], mf_Upper_Gear_Dia=changed_parameter['changed_UGD'], mf_Upper_Bolt_Nummber=Upper_Bolt_Nummber, 
                    mf_Bolt_Orientation='mf_all_random', mf_Bit_Type=random.choice(Bolts_Type), mf_corrosion_Type_Bolt=Bolt_Corrosion_Type, mf_corrosion_Percent_Bolt=changed_parameter['bolt_corrosion_percent'],
                    mf_corrosion_Type_Bottom=Bottom_Corrosion_Type, mf_corrosion_Percent_Bottom=changed_parameter['bottom_corrosion_percent'], save_path=save_dir)
            bpy.data.objects.remove(bpy.data.objects['Motor'])

    elif Extension_Type_A == 'mf_Extension_Type_None' :
        if Bolt_type_mode == 'Torx' :
            bpy.ops.mesh.add_motor(change=True, mf_Color_Render=True, mf_Lower_Gear_Bolt_Random=True,  mf_Upper_Gear_Bolt_Random=True, temp_save=True, mf_Top_Type='mf_Top_Type_A',
                    mf_Extension_Type_A='mf_None', mf_Bottom_Length=changed_parameter['changed_BL'], mf_Sub_Bottom_Length=changed_parameter['changed_SBL'], mf_Lower_Gear_Dia=changed_parameter['changed_LGD'],
                    mf_Lower_Gear_Position=changed_parameter['changed_LGP'], mf_Upper_Gear_Dia=changed_parameter['changed_UGD'], mf_Upper_Bolt_Nummber=Upper_Bolt_Nummber, 
                    mf_Bolt_Orientation='mf_all_random', mf_Bit_Type='mf_Bit_Torx', mf_corrosion_Type_Bolt=Bolt_Corrosion_Type, mf_corrosion_Percent_Bolt=changed_parameter['bolt_corrosion_percent'],
                    mf_corrosion_Type_Bottom=Bottom_Corrosion_Type, mf_corrosion_Percent_Bottom=changed_parameter['bottom_corrosion_percent'], save_path=save_dir)
            bpy.data.objects.remove(bpy.data.objects['Motor'])
        elif Bolt_type_mode == 'Random' :
            bpy.ops.mesh.add_motor(change=True, mf_Color_Render=True, mf_Lower_Gear_Bolt_Random=True,  mf_Upper_Gear_Bolt_Random=True, temp_save=True, mf_Top_Type='mf_Top_Type_A',
                    mf_Extension_Type_A='mf_None', mf_Bottom_Length=changed_parameter['changed_BL'], mf_Sub_Bottom_Length=changed_parameter['changed_SBL'], mf_Lower_Gear_Dia=changed_parameter['changed_LGD'],
                    mf_Lower_Gear_Position=changed_parameter['changed_LGP'], mf_Upper_Gear_Dia=changed_parameter['changed_UGD'], mf_Upper_Bolt_Nummber=Upper_Bolt_Nummber, 
                    mf_Bolt_Orientation='mf_all_random', mf_Bit_Type=random.choice(Bolts_Type), mf_corrosion_Type_Bolt=Bolt_Corrosion_Type, mf_corrosion_Percent_Bolt=changed_parameter['bolt_corrosion_percent'],
                    mf_corrosion_Type_Bottom=Bottom_Corrosion_Type, mf_corrosion_Percent_Bottom=changed_parameter['bottom_corrosion_percent'], save_path=save_dir)
            bpy.data.objects.remove(bpy.data.objects['Motor'])


'''
Value-distribution:
    Type B :
                    Bottom Length = 6.2 - 8.0  step = 0.03
                    Sub Bottom Length = 0.6 - 2  setp = 0.03
                    Lower Gear Dia = 3.5 - 4.5  step = 0.03
                    Lower Gear Position = 3.6 - 4.2
                    Height of Extension left = 6.3 - 7.5
                    Height of Extension right = 3 - 6
                    Bolts Type = 'mf_Bit_Slot', 'mf_Bit_Torx', 'mf_Bit_Corss',
                    Bolt Orientation = 'mf_all_same', 'mf_all_random'
                    Position of Bolt at right = 1.7 - 4    step = 0.03
                    Position of bolt 1 on larger gear = 210 - 225    step = 1
                    Position of bolt 2 on larger gear = 70 - 110    step = 1
The rule I used now:
                    Each changed parameter in changed value + the other parameter in default values  ->  3 groups with different type of bolts
'''
def create_Motor_Obj_B(Extension_Type_B, Bolt_type_mode, changed_parameter, Gear_Bolt_Nummber_B, Bottom_Corrosion_Type, Bolt_Corrosion_Type, save_dir) :         # Motor_type = ['Type_A', 'Type_B']  Type_A_Extension = ['Type_1', 'Type_2']

    try :
        if bpy.data.objects['Cube'] :       # Delete the Cube object and Lamp object from Blender's intialization
            bpy.data.objects.remove(bpy.data.objects['Cube'])
        if bpy.data.objects['Light'] :
            bpy.data.objects.remove(bpy.data.objects['Light'])
    except KeyError :
        pass

    try :
        if bpy.data.objects['Motor']:
            bpy.data.objects.remove(bpy.data.objects['Motor'])
    except KeyError:
        pass

    Bolts_Type = ['mf_Bit_Slot', 'mf_Bit_Torx', 'mf_Bit_Corss']
    if Extension_Type_B == 'Extension_Type_1' :
        if Bolt_type_mode == 'Torx' :
            bpy.ops.mesh.add_motor(change=True, mf_Color_Render=True, mf_Gear_Bolt_Random_B=True, temp_save=True, mf_Top_Type='mf_Top_Type_B',
                mf_Extension_Type_B='mf_Extension_Type_1', mf_Bottom_Length=changed_parameter['changed_BL'], mf_Sub_Bottom_Length=changed_parameter['changed_SBL'], mf_Lower_Gear_Dia=changed_parameter['changed_LGD'], 
                mf_Lower_Gear_Position=changed_parameter['changed_LGP'], mf_Gear_Bolt_Nummber_B='2', mf_Type_B_Height_1=changed_parameter['changed_HEL'], 
                mf_Type_B_Height_2=changed_parameter['changed_HER'], mf_Gear_Bolt_Right_B=changed_parameter['changed_GBR'],
                mf_Bolt_Orientation='mf_all_random', mf_Bit_Type='mf_Bit_Torx', mf_corrosion_Type_Bolt=Bolt_Corrosion_Type, mf_corrosion_Percent_Bolt=changed_parameter['bolt_corrosion_percent'],
                    mf_corrosion_Type_Bottom=Bottom_Corrosion_Type, mf_corrosion_Percent_Bottom=changed_parameter['bottom_corrosion_percent'], save_path=save_dir)
            bpy.data.objects.remove(bpy.data.objects['Motor'])
        elif Bolt_type_mode == 'random' :
            bpy.ops.mesh.add_motor(change=True, mf_Color_Render=True, mf_Gear_Bolt_Random_B=True, temp_save=True, mf_Top_Type='mf_Top_Type_B',
                mf_Extension_Type_B='mf_Extension_Type_1', mf_Bottom_Length=changed_parameter['changed_BL'], mf_Sub_Bottom_Length=changed_parameter['changed_SBL'], mf_Lower_Gear_Dia=changed_parameter['changed_LGD'], 
                mf_Lower_Gear_Position=changed_parameter['changed_LGP'], mf_Gear_Bolt_Nummber_B='2', mf_Type_B_Height_1=changed_parameter['changed_HEL'], 
                mf_Type_B_Height_2=changed_parameter['changed_HER'], mf_Gear_Bolt_Right_B=changed_parameter['changed_GBR'],
                mf_Bolt_Orientation='mf_all_random', mf_Bit_Type=random.choice(Bolts_Type), mf_corrosion_Type_Bolt=Bolt_Corrosion_Type, mf_corrosion_Percent_Bolt=changed_parameter['bolt_corrosion_percent'],
                    mf_corrosion_Type_Bottom=Bottom_Corrosion_Type, mf_corrosion_Percent_Bottom=changed_parameter['bottom_corrosion_percent'], save_path=save_dir)
            bpy.data.objects.remove(bpy.data.objects['Motor'])
    
    elif Extension_Type_B == 'Extension_Type_None' :
        if Bolt_type_mode == 'Torx' :
            bpy.ops.mesh.add_motor(change=True, mf_Color_Render=True, mf_Gear_Bolt_Random_B=True, temp_save=True, mf_Top_Type='mf_Top_Type_B',
                mf_Extension_Type_B='mf_None', mf_Bottom_Length=changed_parameter['changed_BL'], mf_Sub_Bottom_Length=changed_parameter['changed_SBL'], mf_Lower_Gear_Dia=changed_parameter['changed_LGD'], 
                mf_Lower_Gear_Position=changed_parameter['changed_LGP'], mf_Gear_Bolt_Nummber_B=Gear_Bolt_Nummber_B, mf_Type_B_Height_1=changed_parameter['changed_HEL'], 
                mf_Type_B_Height_2=changed_parameter['changed_HER'], mf_Gear_Bolt_Right_B=changed_parameter['changed_GBR'],
                mf_Bolt_Orientation='mf_all_random', mf_Bit_Type='mf_Bit_Torx', mf_corrosion_Type_Bolt=Bolt_Corrosion_Type, mf_corrosion_Percent_Bolt=changed_parameter['bolt_corrosion_percent'],
                    mf_corrosion_Type_Bottom=Bottom_Corrosion_Type, mf_corrosion_Percent_Bottom=changed_parameter['bottom_corrosion_percent'], save_path=save_dir)
            bpy.data.objects.remove(bpy.data.objects['Motor'])
        elif Bolt_type_mode == 'Random' :
            bpy.ops.mesh.add_motor(change=True, mf_Color_Render=True, mf_Gear_Bolt_Random_B=True, temp_save=True, mf_Top_Type='mf_Top_Type_B',
                mf_Extension_Type_B='mf_None', mf_Bottom_Length=changed_parameter['changed_BL'], mf_Sub_Bottom_Length=changed_parameter['changed_SBL'], mf_Lower_Gear_Dia=changed_parameter['changed_LGD'], 
                mf_Lower_Gear_Position=changed_parameter['changed_LGP'], mf_Gear_Bolt_Nummber_B=Gear_Bolt_Nummber_B, mf_Type_B_Height_1=changed_parameter['changed_HEL'], 
                mf_Type_B_Height_2=changed_parameter['changed_HER'], mf_Gear_Bolt_Right_B=changed_parameter['changed_GBR'],
                mf_Bolt_Orientation='mf_all_random', mf_Bit_Type=random.choice(Bolts_Type), mf_corrosion_Type_Bolt=Bolt_Corrosion_Type, mf_corrosion_Percent_Bolt=changed_parameter['bolt_corrosion_percent'],
                    mf_corrosion_Type_Bottom=Bottom_Corrosion_Type, mf_corrosion_Percent_Bottom=changed_parameter['bottom_corrosion_percent'], save_path=save_dir)
            bpy.data.objects.remove(bpy.data.objects['Motor'])

def main(num_motor):
    BASE_DIR='E:\\Motor_1000\\'
    save_dir_TypeA1_origial = BASE_DIR+'TypeA1'
    if not os.path.exists(save_dir_TypeA1_origial):
        os.makedirs(save_dir_TypeA1_origial)

    save_dir_TypeA2_original = BASE_DIR+'TypeA2'
    if not os.path.exists(save_dir_TypeA2_original):
        os.makedirs(save_dir_TypeA2_original)

    save_dir_TypeANone_original = BASE_DIR+'TypeA0'
    if not os.path.exists(save_dir_TypeANone_original):
        os.makedirs(save_dir_TypeANone_original)

    save_dir_TypeB1_origial = BASE_DIR+'TypeB1'
    if not os.path.exists(save_dir_TypeB1_origial):
        os.makedirs(save_dir_TypeB1_origial)

    save_dir_TypeBNone_origial = BASE_DIR+'TypeB0'
    if not os.path.exists(save_dir_TypeBNone_origial):
        os.makedirs(save_dir_TypeBNone_origial)


    Upper_Bolt_Nummber_A = ['1', '2','2','3']
    Upper_Bolt_Nummber_B = ['2','3']
    # Corrosion_Type = ['None', 'None', 'None', 'None', 'None', 'None', 'Rust 1', 'Rust 2',  
    #                   'Rust 3', 'Rust 4', 'Rust 5', 'Rust 6', 'Rust 7', 'Rust 8', 'Rust 9']
    Corrosion_Type = ['None']
    
    for _ in range(int(num_motor)):
        changed_parameter_A = {}
        # changed_parameter_A['changed_BL'] = float(np.random.uniform(6.2, 8.0, 1))          # Bottom_Length
        # changed_parameter_A['changed_SBL'] = float(np.random.uniform(0.6, 2.0, 1))         # Sub_Bottom_Length
        # changed_parameter_A['changed_LGD'] = float(np.random.uniform(3.5, 4.5, 1))         # Lower_Gear_Diameter
        # changed_parameter_A['changed_LGP'] = float(np.random.uniform(3.6, 4.2, 1))         # Lower_Gear_Position
        # changed_parameter_A['changed_UGD'] = float(np.random.uniform(5, 6.5, 1)) 
        # changed_parameter_A['bolt_corrosion_percent'] = int(np.random.uniform(20, 40))
        # changed_parameter_A['bottom_corrosion_percent'] = int(np.random.uniform(20, 40))
        # num_bolts_A=random.choice(Upper_Bolt_Nummber_A)
        # Bottom_Corrosion=random.choice(Corrosion_Type)
        # Bolt_Corrosion=random.choice(Corrosion_Type)
        ###########################################  
        #   Create Type A Extension 1 Motor       #
        ###########################################
        # create_Motor_Obj_A(Extension_Type_A = 'mf_Extension_Type_1', Bolt_type_mode = 'Torx', changed_parameter = changed_parameter_A, 
        #     Upper_Bolt_Nummber=num_bolts_A, Bottom_Corrosion_Type=Bottom_Corrosion, Bolt_Corrosion_Type=Bolt_Corrosion, save_dir = save_dir_TypeA1_origial)

        ###########################################  
        #   Create Type A Extension 2 Motor       #
        ###########################################

        # changed_parameter_A['changed_BL'] = float(np.random.uniform(6.2, 8.0, 1))          # Bottom_Length
        # changed_parameter_A['changed_SBL'] = float(np.random.uniform(0.6, 2.0, 1))         # Sub_Bottom_Length
        # changed_parameter_A['changed_LGD'] = float(np.random.uniform(3.5, 4.5, 1))         # Lower_Gear_Diameter
        # changed_parameter_A['changed_LGP'] = float(np.random.uniform(3.6, 4.2, 1))         # Lower_Gear_Position
        # changed_parameter_A['changed_UGD'] = float(np.random.uniform(5, 6.5, 1))
        # changed_parameter_A['bolt_corrosion_percent'] = int(np.random.uniform(20, 40))
        # changed_parameter_A['bottom_corrosion_percent'] = int(np.random.uniform(20, 40)) 
        # num_bolts_A=random.choice(Upper_Bolt_Nummber_A)
        # Bottom_Corrosion=random.choice(Corrosion_Type)
        # Bolt_Corrosion=random.choice(Corrosion_Type)
        # create_Motor_Obj_A(Extension_Type_A = 'mf_Extension_Type_2', Bolt_type_mode = 'Torx', changed_parameter = changed_parameter_A, 
        #     Upper_Bolt_Nummber=num_bolts_A, Bottom_Corrosion_Type=Bottom_Corrosion, Bolt_Corrosion_Type=Bolt_Corrosion, save_dir =save_dir_TypeA2_original)

        ###########################################  
        #   Create Type A Extension None Motor    #
        ###########################################

        # changed_parameter_A['changed_BL'] = float(np.random.uniform(6.2, 8.0, 1))          # Bottom_Length
        # changed_parameter_A['changed_SBL'] = float(np.random.uniform(0.6, 2.0, 1))         # Sub_Bottom_Length
        # changed_parameter_A['changed_LGD'] = float(np.random.uniform(3.5, 4.5, 1))         # Lower_Gear_Diameter
        # changed_parameter_A['changed_LGP'] = float(np.random.uniform(3.6, 4.2, 1))         # Lower_Gear_Position
        # changed_parameter_A['changed_UGD'] = float(np.random.uniform(5, 6.5, 1)) 
        # changed_parameter_A['bolt_corrosion_percent'] = int(np.random.uniform(20, 40))
        # changed_parameter_A['bottom_corrosion_percent'] = int(np.random.uniform(20, 40))
        # create_Motor_Obj_A(Extension_Type_A = 'mf_Extension_Type_None', Bolt_type_mode = 'Torx', changed_parameter = changed_parameter_A, 
        #     Upper_Bolt_Nummber=num_bolts_A, Bottom_Corrosion_Type=Bottom_Corrosion, Bolt_Corrosion_Type=Bolt_Corrosion, save_dir = save_dir_TypeANone_original)

        ###########################################  
        #   Create Type B Extension 1 Motor       #
        ###########################################

        changed_parameter = {}
        # changed_parameter['changed_BL'] = float(np.random.uniform(6.2, 8.0, 1))          # Bottom_Length
        # changed_parameter['changed_SBL'] = float(np.random.uniform(0.6, 2.0, 1))         # Sub_Bottom_Length
        # changed_parameter['changed_LGD'] = float(np.random.uniform(3.5, 4.5, 1))         # Lower_Gear_Diameter
        # changed_parameter['changed_LGP'] = float(np.random.uniform(3.6, 4.2, 1))         # Lower_Gear_Position
        # changed_parameter['changed_GBR'] = float(np.random.uniform(1.7, 4.0, 1))         # Gear_Bolt_Right_B
        # changed_parameter['changed_HEL'] = float(np.random.uniform(6.3, 7.5, 1))         # Height of Extension left
        # changed_parameter['changed_HER'] = float(np.random.uniform(3.0, 6.0, 1))         # Height of Extension right
        # changed_parameter['bolt_corrosion_percent'] = int(np.random.uniform(20, 40))
        # changed_parameter['bottom_corrosion_percent'] = int(np.random.uniform(20, 40))
        num_bolts_B=random.choice(Upper_Bolt_Nummber_B)
        
        # create_Motor_Obj_B(Extension_Type_B = 'Extension_Type_1', Bolt_type_mode = 'Torx', changed_parameter = changed_parameter, 
        #     Gear_Bolt_Nummber_B= Upper_Bolt_Nummber_B[0] , Bottom_Corrosion_Type=Bottom_Corrosion, Bolt_Corrosion_Type=Bolt_Corrosion, save_dir = save_dir_TypeB1_origial)      # Bolt_type_mode = ['Random', 'Torx']

        ###########################################  
        #   Create Type B Extension None Motor    #
        ###########################################
        
        changed_parameter['changed_BL'] = float(np.random.uniform(6.2, 8.0, 1))          # Bottom_Length
        changed_parameter['changed_SBL'] = float(np.random.uniform(0.6, 2.0, 1))         # Sub_Bottom_Length
        changed_parameter['changed_LGD'] = float(np.random.uniform(3.5, 4.5, 1))         # Lower_Gear_Diameter
        changed_parameter['changed_LGP'] = float(np.random.uniform(3.6, 4.2, 1))         # Lower_Gear_Position
        changed_parameter['changed_GBR'] = float(np.random.uniform(1.7, 4.0, 1))         # Gear_Bolt_Right_B
        changed_parameter['changed_HEL'] = float(np.random.uniform(6.3, 7.5, 1))         # Height of Extension left
        changed_parameter['changed_HER'] = float(np.random.uniform(3.0, 6.0, 1))         # Height of Extension right
        changed_parameter['bolt_corrosion_percent'] = int(np.random.uniform(20, 40))
        changed_parameter['bottom_corrosion_percent'] = int(np.random.uniform(20, 40))
        create_Motor_Obj_B(Extension_Type_B = 'Extension_Type_None', Bolt_type_mode = 'Torx', changed_parameter = changed_parameter,
            Gear_Bolt_Nummber_B= num_bolts_B, Bottom_Corrosion_Type=Bottom_Corrosion, Bolt_Corrosion_Type=Bolt_Corrosion, save_dir = save_dir_TypeBNone_origial)



if __name__ == '__main__':
    num_motor_for_dataset=200      #   paramater to set dataset size
    main(num_motor=num_motor_for_dataset)