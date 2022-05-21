from warnings import catch_warnings
import torch
import pathlib
import torchvision.models
from datetime import datetime
from itertools import compress

import copy
import json


from typing import Dict
from typing import Any



torch.autograd.set_detect_anomaly(True)


TORCH_DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"[Torch]  Using '{TORCH_DEVICE}' as device.")

_stamp = datetime.now().strftime("%Y_%m_%d___%H_%M_%S")

ROOT_DPATH: pathlib.Path = pathlib.Path(__file__).parent.parent.resolve()
OUTPUT_DPATH: pathlib.Path = ROOT_DPATH/'output'
EXPERIMENT_DPATH: pathlib.Path = OUTPUT_DPATH/datetime.now().strftime('%Y_%m_%d___%H_%M_%S')
[x.rmdir() for x in EXPERIMENT_DPATH.parent.resolve().iterdir() if x.is_dir() and not any(x.iterdir())]
EXPERIMENT_DPATH.mkdir(parents=True, exist_ok=True)




user_input = {
    'ann': {
        'cnn': {
            'torchvision_model_id': {
                0: 'resnet8',
                1: 'resnet18',
                2: 'resnet50',
                3: 'efficientnet_b0',
                4: 'efficientnet_b5',
                5: 'mobilenet_v3_small'
            }[1],
            'pretrained': True,
            'trainable': True,
        },
        'cat': {
            #'enabled': True,
            #'input_size': 6
        },
        'gru': {
            #'enabled': True,
            'hidden_size': 128,
            'num_layers': 1,
            'dropout': 0.0, #0.2
        },
        'fc': {
            'enabled': True,
            'width': 256,
            'num_layers': 1,
            'activation_function_id': {
                0: 'ReLU',
                1: 'PReLU',
            }[0],
            'dropout': 0.5,
        },
        'head': {
            #'output_size': 3
            'activation_function_id': {
                0: 'ReLU',
                1: 'PReLU',
            }[0],
        }
    },
    'data': {
        'inputs': [
            'image',
            #'imu',
        ],
        'label': {
            0: 'waypoint_speed', 
            1: 'control_command'
        }[0],
        'image': {
            'resize_factor': 1/2
        },
        'sequential': {
            'enabled': True,
            'length': 3, #200#36#18#72,
            'step': 2
        },
        'racetrack_types': [
            'figure_8_racetrack',
            #'intermediate_target_loss',
        ],
        'unity_scenes': {
            'train': [
                #'spaceship_interior',
                'destroyed_city',
                #'industrial_park',
                #'polygon_city',
                #'desert_mountain'
            ],
            'valid': [
                #'spaceship_interior',
                'destroyed_city',
                #'industrial_park',
                #'polygon_city',
                #'desert_mountain'
            ]
        },
        'num_samples_per_run': None
    },
    'learn': {
        'batch_size': 16, #4#8#16#4
        'num_epochs': 100,
        'optimizer_id': {
            0: 'Adam',
            1: 'SGD'
        }[0],
        'loss': {
            'id': {
                0: 'SmoothL1Loss',
                1: 'MSELoss'
            }[0],
            'waypoint_to_speed_weighting': 10
        },
        'learning_rate' : {
            'initial_value': 1e-4,
            'scheduler_id': 'ExponentialLR',
            'gamma': 0.99
        }
    }
}





def assert_user_input():
    pass


def get_configuration (user_input: Dict[str, Dict[str, Any]]) -> Dict[Any, Any]:

    
    assert_user_input()
    params = copy.deepcopy(user_input)

    


    ##### HARD CODED STUFF
    ##############################
    params['data']['dirname'] = 'data'
    params['data']['image']['width'] = dict()
    params['data']['image']['width']['original'] = 720
    params['data']['image']['height'] = dict()
    params['data']['image']['height']['original'] = 480
    params['data']['image']['num_channels'] = 3
    params['data']['image']['frame_rate'] = 50.0
    
    params['data']['raw'] = {
        'dirname': 'raw',
        'run_dir_file_names': {
            'image': 'images',
            'imu': 'imu.txt',
            'waypoint_speed': 'labels.txt',
            'control_command': 'control_command.txt'
        },
        'racetrack_types': [
            'figure_8_racetrack',
            'intermediate_target_loss'
        ],
        'unity_scenes': [
            'spaceship_interior',
            'destroyed_city',
            'industrial_park',
            'polygon_city',
            'desert_mountain'
        ],
        'scene_sites': [
            'a',
            'b',
            'c'
        ],
        'gate_types': [
            'tub_dai',
            'thu_dme'
        ],
        'repetitions': {
            'figure_8_racetrack': 3,
            'intermediate_target_loss': 1
        },
        'gap_types': {
            'figure_8_racetrack': [
                None
            ],
            'intermediate_target_loss': [
                'narrow',
                'wide'
            ],
        },
        'directions': {
            'figure_8_racetrack': [
                None
            ],
            'intermediate_target_loss': [
                'clockwise',
                'counterclockwise'
            ],
        },
        'image': {
            'width': 720,
            'height': 480,
            'num_channels': 3
        }
    }

    params['data']['processed'] = {
        'dirname': 'processed',
        'filename': {
            'prefix': 'run_',
            'extension': '.csv',
            'identifier': {
                'figure_8_racetrack': lambda run_file_list: [x for x in run_file_list if not 'X' in x.stem],
                'intermediate_target_loss': lambda run_file_list: [x for x in run_file_list if 'X' in x.stem]
            }
        },
        'columns': {
            'run': [
                'run_id'
            ],
            'image': [
                'img_filepath'
            ],
            'imu': [
                'imu_t',
                'imu_lin_acc_x',
                'imu_lin_acc_y', 
                'imu_lin_acc_z', 
                'imu_ang_vel_x', 
                'imu_ang_vel_y', 
                'imu_ang_vel_z'
            ],
            'waypoint_speed': [
                'wps_t', 
                'wps_x_img', 
                'wps_y_img', 
                'wps_v_norm'
            ],
            'control_command': [
                'cmd_t', 
                'cmd_ang_vel_x', 
                'cmd_ang_vel_y', 
                'cmd_ang_vel_z', 
                'cmd_ang_acc_x', 
                'cmd_ang_acc_y', 
                'cmd_ang_acc_z', 
                'cmd_collective_thrust'
            ],
        },
    }

    params['data']['loaded'] = {
        'columns': {
            'image': params['data']['processed']['columns']['image'],
            'imu': params['data']['processed']['columns']['imu'][1:],
        },
        'image' : {
            'num_channels': params['data']['image']['num_channels']
        }
    }


    ##############################



    # DERIVED STUFF
    ##############################
    for data_input in params['data']['inputs']:
        if data_input not in ['image', 'imu']:
            raise ValueError(f"Unknown user input > data input: {data_input}.")
    if not 'image' in params['data']['inputs']:
        raise ValueError(f"Data inputs: {params['data']['inputs']} but 'image' is required.")
    if 'imu' in params['data']['inputs']:
        params['ann']['cat']['enabled'] = True
        params['ann']['cat']['input_size'] = 6
    else:
        params['ann']['cat']['enabled'] = False
        params['ann']['cat']['input_size'] = None
        params['data']['loaded']['columns']['imu'] = []

    

    data_labels_dict = {
        'waypoint_speed': 3,
        'control_command': 7,
    }
    try:
        params['ann']['head']['output_size'] = data_labels_dict[params['data']['label']]
    except:
        raise ValueError(f"Unknown data labels: {params['data']['label']}.")

    if params['data']['sequential']['enabled']:
        params['ann']['gru']['enabled'] = True
        params['learn']['learning_rate']['gamma'] **= 12
    else:
        params['data']['sequential']['length'] = 1
        params['data']['sequential']['step'] = 1
        params['ann']['gru']['enabled'] = False
        #params['ann']['gru']['hidden_size'] = None
        #params['ann']['gru']['num_layers'] = None
        #params['ann']['gru']['dropout'] = None

    if params['ann']['fc']['enabled']:
        pass
    else:
        params['ann']['fc']['width'] = None
        params['ann']['fc']['num_layers'] = None
        params['ann']['fc']['activation_function_id'] = None
        params['ann']['fc']['dropout'] = None

    

    try:
        rf = params['data']['image']['resize_factor']
        w_raw = params['data']['raw']['image']['width']
        h_raw = params['data']['raw']['image']['height']
        params['data']['loaded']['image']['width'] = int(w_raw * rf)
        params['data']['loaded']['image']['height'] = int(h_raw * rf)
    except:
        raise AssertionError(f"Failed to set resized width and height of image.")
    


    try:
        params['data']['loaded']['columns']['label'] = {
            'waypoint_speed': params['data']['processed']['columns']['waypoint_speed'][1:], 
            'control_command': params['data']['processed']['columns']['control_command'][1:]
        }[params['data']['label']]
    except:
        raise AssertionError(f"Failed to set loaded label columns.")

    import copy
    return params




def get_root_dpath () -> pathlib.Path:
    return pathlib.Path(__file__).parent.parent.resolve()

def get_experiment_dpath () -> pathlib.Path:
    return get_root_dpath()/f"output/{_stamp}"


if __name__ == "__main__":

    print(get_root_dpath())
    #params = get_parameters()
    #print(json.dumps(params, sort_keys=True, indent=4))






'''
DEBUG = False

# --- ANN ---

# CNN
#CNN_MODEL_ID = 'resnet18'
#CNN_MODEL_ID = 'resnet50'
#CNN_MODEL_ID = 'efficientnet_b0'
#CNN_MODEL_ID = 'efficientnet_b5'
CNN_MODEL_ID = 'mobilenet_v3_small'
CNN_PRETRAINED = True
CNN_FROZEN = False
CNN_DROPOUT = 0.0

# RNN
USE_RNN = True
RNN_TYPE = 'GRU'
RNN_SEQUENCE_STEP = 10
RNN_SEQUENCE_LEN = 20#200#36#18#72
GRU_HIDDEN_SIZE = 128
GRU_NUM_LAYERS = 1
GRU_DROPOUT = 0.0#0.2

# FC
USE_FC = False
FC_OUTPUT_SIZE = 128
FC_NUM_LAYERS = 3
FC_ACTIVATION_FCT_ID = 'PReLU'
FC_DROPOUT = 0.2



# --- DATA ---


#RUN_SELECTION_MODE = 'OVERFIT_SINGLE_RANDOM_RUN'
RUN_SELECTION_MODE = 'TRAIN_AND_VALIDATE_ON_SCENE_12345'#1234'
#RUN_SELECTION_MODE = 'TRAIN_ON_SCENE_04_AND_VALIDATE_ON_SCENE_123'
#RUN_SELECTION_MODE = 'TRAIN_AND_CROSS_VALIDATE_OVER_SCENES'
RUN_USE_FIG8 = True
RUN_USE_ITL = False
#SCENE_TRAIN_RUN_N = 'ALL'
#SCENE_VALID_RUN_N = 6#'ALL'
RUN_IMAGE_N = 'ALL'

IMAGE_RESIZE_FACTOR = 1/4
INPUT_USE_IMU = True
LABELS_USE_WAYPOINT = True
LABELS_USE_SPEED = True
LABELS_USE_CTRL_CMD = False



# --- LEARN ---

BATCH_SIZE = 4#4#8#16#4
EPOCH_N = 100
OPTIMIZER_TYPE = 'Adam'
#OPTIMIZER_TYPE = 'SGD'
LEARN_RATE_INITIAL = 1e-4
LEARN_RATE_SCHEDULER_TYPE = 'ExponentialLR'
LEARN_RATE_GAMMA = 0.99 **12 if USE_RNN else 0.99
LOSS_TYPE = 'SmoothL1Loss'
#LOSS_TYPE = 'MSELoss'
LOSS_SPEED2WAYPOINT_RATIO = 1e-1'''
































"""#################################
########## HARD CODED ###########
#################################

# PATHS
PROJECT_DIR = pathlib.Path(__file__).parent.parent.resolve()
SRC_DIR = PROJECT_DIR/"src"
INPUT_DIR = PROJECT_DIR/"input"
OUTPUT_DIR = PROJECT_DIR/"output"
EXP_DIR = OUTPUT_DIR/datetime.now().strftime("%Y_%m_%d___%H_%M_%S")

# RUN FILES
RUN_FILES_RUN_TYPES = ['figure_8_racetrack', 'intermediate_target_loss']
RUN_FILES_SCENES = ['spaceship_interior', 'destroyed_city', 'industrial_park', 'polygon_city', 'desert_mountain']
RUN_FILES_SITE_N = 3
RUN_FILES_GATE_N = 2
RUN_FILES_FIG8_REPETITION_N = 3
RUN_FILES_ITL_CURVATURE_N = 2
RUN_FILES_ITL_DIRECTION_N = 2
RUN_FILES_NAME_PREFIX = "run_"
RUN_FILES_EXTENSION = ".csv"


# DATA
IMAGE_WIDTH_ORIGINAL = 720
IMAGE_HEIGHT_ORIGINAL = 480
IMAGE_FRAMES_PER_SECOND = 50

# COLUMNS IN RUN FILES
COLS_RUN_FILE_IMAGE = ['img_file_path']
COLS_RUN_FILE_IMU = ['imu_t', 'lin_acc_x', 'lin_acc_y', 'lin_acc_z', 'ang_vel_x', 'ang_vel_y', 'ang_vel_z']
COLS_RUN_FILE_STANDARD_LABEL = ['std_labels_t', 'wayp_img_x', 'wayp_img_y', 'norm_speed']
COLS_RUN_FILE_CONTROL_COMMAND = ['ctrl_cmds_t', 'ang_vel_x', 'ang_vel_y', 'ang_vel_z', 'ang_acc_x', 'ang_acc_y', 'ang_acc_z', 'collective_thrust']

# COLUMNS POTENTIALLY USED
COLS_INPUT_IMAGE_FILE = COLS_RUN_FILE_IMAGE
COLS_INPUT_IMU = COLS_RUN_FILE_IMU[1:]
COLS_LABELS_WAYPOINT = COLS_RUN_FILE_STANDARD_LABEL[1:3]
COLS_LABELS_SPEED = [COLS_RUN_FILE_STANDARD_LABEL[3]]
COLS_LABELS_CTRL_CMD = COLS_RUN_FILE_CONTROL_COMMAND[1:]

# ANN
CNN_MODELS_WITH_OUTPUT_SIZE = {
    'resnet18': (
        torch.nn.Sequential(*list(torchvision.models.resnet18(pretrained=CNN_PRETRAINED).children())[:-1]), 
        512
    ),
    'resnet50': (
        torch.nn.Sequential(*list(torchvision.models.resnet50(pretrained=CNN_PRETRAINED).children())[:-1]), 
        2048
    ),
    'efficientnet_b0': (
        torch.nn.Sequential(*list(torchvision.models.efficientnet_b0(pretrained=CNN_PRETRAINED).children())[:-1]), 
        1280
    ),
    'efficientnet_b5': (
        torch.nn.Sequential(*list(torchvision.models.efficientnet_b5(pretrained=CNN_PRETRAINED).children())[:-1]), 
        2048
    ),
    'mobilenet_v3_small': (
        torch.nn.Sequential(*list(torchvision.models.mobilenet_v3_small(pretrained=CNN_PRETRAINED).children())[:-1]),
        576
    ),
    #"convnext_tiny": (
    #    torch.nn.Sequential(*list(torchvision.models.convnext_tiny(pretrained=CNN_PRETRAINED).children())[:-1]),
    #    576
    #),
    
    
}

GRU_BIAS            = True
GRU_BATCH_FIRST     = False
GRU_BIDIRECTIONAL   = False
HEAD_BIAS         = True
INPUT_USE_IMAGE = True




##############################
########## DERIVED ###########
##############################


# PyTorch Configuration
TORCH_DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Torch using '{TORCH_DEVICE}' as device.")

# Image Resizing
IMAGE_WIDTH_RESIZED = int(IMAGE_WIDTH_ORIGINAL * IMAGE_RESIZE_FACTOR)
IMAGE_HEIGHT_RESIZED = int(IMAGE_HEIGHT_ORIGINAL * IMAGE_RESIZE_FACTOR)


SHUFFLE_TRAIN_DATA = True if not USE_RNN else False


# Input-Output-Combinations
#
# Input: 
#   - image (mandatory) + IMU (optionally)
#
# Output:
#   - waypoint (mandatory) + normalized speed (optionally)
#   or
#   - control commands (mandatory)

IMG__2__WP = True\
    and INPUT_USE_IMAGE\
    and not INPUT_USE_IMU\
    and LABELS_USE_WAYPOINT\
    and not LABELS_USE_SPEED\
    and not LABELS_USE_CTRL_CMD

IMG__2__WP_SPEED = True\
    and INPUT_USE_IMAGE\
    and not INPUT_USE_IMU\
    and LABELS_USE_WAYPOINT\
    and LABELS_USE_SPEED\
    and not LABELS_USE_CTRL_CMD

IMG__2__CTRLCMD = True\
    and INPUT_USE_IMAGE\
    and not INPUT_USE_IMU\
    and not LABELS_USE_WAYPOINT\
    and not LABELS_USE_SPEED\
    and LABELS_USE_CTRL_CMD

IMG_IMU__2__WP = True\
    and INPUT_USE_IMAGE\
    and INPUT_USE_IMU\
    and LABELS_USE_WAYPOINT\
    and not LABELS_USE_SPEED\
    and not LABELS_USE_CTRL_CMD

IMG_IMU__2__WP_SPEED = True\
    and INPUT_USE_IMAGE\
    and INPUT_USE_IMU\
    and LABELS_USE_WAYPOINT\
    and LABELS_USE_SPEED\
    and not LABELS_USE_CTRL_CMD

IMG_IMU__2__CTRLCMD = True\
    and INPUT_USE_IMAGE\
    and INPUT_USE_IMU\
    and not LABELS_USE_WAYPOINT\
    and not LABELS_USE_SPEED\
    and LABELS_USE_CTRL_CMD

if\
not IMG__2__WP and\
not IMG__2__WP_SPEED and\
not IMG__2__CTRLCMD and\
not IMG_IMU__2__WP and\
not IMG_IMU__2__WP_SPEED and\
not IMG_IMU__2__CTRLCMD:
    error_msg = "No implementation for specified input-output combination:\n"\
        + "\n"\
        + f"\t'INPUT_USE_IMAGE':     {INPUT_USE_IMAGE}\n"\
        + f"\t'INPUT_USE_IMU':       {INPUT_USE_IMU}\n"\
        + "\n"\
        + f"\t'LABELS_USE_WAYPOINT': {LABELS_USE_WAYPOINT}\n"\
        + f"\t'LABELS_USE_SPEED':    {LABELS_USE_SPEED}\n"\
        + f"\t'LABELS_USE_CTRL_CMD': {LABELS_USE_CTRL_CMD}"
    raise ValueError(error_msg)


IO_combination_count = 0
if IMG__2__WP: IO_combination_count += 1
if IMG__2__WP_SPEED: IO_combination_count += 1
if IMG__2__CTRLCMD: IO_combination_count += 1
if IMG_IMU__2__WP: IO_combination_count += 1
if IMG_IMU__2__WP_SPEED: IO_combination_count += 1
if IMG_IMU__2__CTRLCMD: IO_combination_count += 1
if IO_combination_count != 1:
    error_msg = "Erroneous implementation:"\
        + f"\tNumber of activated input-output combinations: {IO_combination_count} [Required: =1]"
    raise AssertionError(error_msg)




# Set inputs ...
if IMG__2__WP or IMG__2__WP_SPEED or IMG__2__CTRLCMD:
    COLS_INPUTS = COLS_INPUT_IMAGE_FILE
if IMG_IMU__2__WP or IMG_IMU__2__WP_SPEED or IMG_IMU__2__CTRLCMD:
    COLS_INPUTS = COLS_INPUT_IMAGE_FILE + COLS_INPUT_IMU

# ... and labels
if IMG__2__WP or IMG_IMU__2__WP:
    COLS_LABELS = COLS_LABELS_WAYPOINT
if IMG__2__WP_SPEED or IMG_IMU__2__WP_SPEED:
    COLS_LABELS = COLS_LABELS_WAYPOINT + COLS_LABELS_SPEED
if IMG__2__CTRLCMD or IMG_IMU__2__CTRLCMD:
    COLS_LABELS = COLS_LABELS_CTRL_CMD

COLS_INPUTS_AND_LABELS = COLS_INPUTS + COLS_LABELS

INPUT_N = len(COLS_INPUTS)
LABEL_N = len(COLS_LABELS)






[x.rmdir() for x in EXP_DIR.parent.resolve().iterdir() if x.is_dir() and not any(x.iterdir())]
EXP_DIR.mkdir(parents=True, exist_ok=True)






if DEBUG:
    RUN_SELECTION_MODE = 'OVERFIT_SINGLE_RANDOM_RUN'
    #RUN_SELECTION_MODE = 'TRAIN_AND_VALIDATE_ON_SCENE_1'#1234'
    #RUN_SELECTION_MODE = 'TRAIN_ON_SCENE_04_AND_VALIDATE_ON_SCENE_123'
    #RUN_SELECTION_MODE = 'TRAIN_AND_CROSS_VALIDATE_OVER_SCENES'
    #RUN_USE_ITL = False
    #SCENE_TRAIN_RUN_N = 'ALL'
    #SCENE_VALID_RUN_N = 1#'ALL'
    RUN_IMAGE_N = 100



if __name__ == "__main__":
    
    cnn_keys = sorted(CNN_MODELS_WITH_OUTPUT_SIZE.keys())
    for key in cnn_keys:
        model, out_size = CNN_MODELS_WITH_OUTPUT_SIZE[key]
        mem_params = sum([param.nelement()*param.element_size() for param in model.parameters()])
        mem_bufs = sum([buf.nelement()*buf.element_size() for buf in model.buffers()])
        mem = (mem_params + mem_bufs) / 2**20
        print(f'  {key}')
        print(f'    {mem} MB')
        print(f'    {out_size} Outputs')"""