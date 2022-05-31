# This file is only an example for the git repo and should not be modified.
# Make a copy of this file named 'user_input.py' and modify parameters there.

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

        },
        'gru': {
            'num_layers': 1,
            'hidden_size': 512,
            'dropout': 0.0, #0.2
        },
        'fc': {
            'num_layers': 1, #None
            'width': 512,
            'activation_function_id': {
                0: 'ReLU',
                1: 'PReLU',
            }[0],
            'dropout': 0.5,
        },
        'head': {
            'activation_function_id': {
                0: 'ReLU',
                1: 'PReLU',
            }[0],
        }
    },

    'data': {
        'sequential': {
            'length': 10, #200#36#18#72,
            'step': 1
        },
        'processed': {
            'rgb': {
                'resize_factor': 1/2,
            },
        },
        'input': {
            'cnn' : [
                'rgb_fpath'
            ],
            'cat' : [
                'rgb_dt',
                'imu_dt',
                'imu_linacc_x',
                'imu_linacc_y',
                'imu_linacc_z',
                'imu_angvel_x',
                'imu_angvel_y',
                'imu_angvel_z'
            ],
        },
        'label': [
            'exp_waypoint_x',
            'exp_waypoint_y',
            'exp_normspeed',
            #'ctrlcmd_bodyrates_x',
            #'ctrlcmd_bodyrates_y',
            #'ctrlcmd_bodyrates_z',
            #'ctrlcmd_angacc_x',
            #'ctrlcmd_angacc_y',
            #'ctrlcmd_angacc_z',
            #'ctrlcmd_collthrust'
        ],
    },

    'learn': {
        'batch_size': 4, #4#8#16#4
        'num_epochs': 10,
        'optimizer': {
            'id': {
                0: 'Adam',
                1: 'SGD'
            }[0],
        },
        'loss': {
            'id': {
                0: 'SmoothL1Loss',
                1: 'MSELoss'
            }[0],
            'waypoint_to_speed_weighting': 10 # NOT IMPLEMENTED
        },
        'lr_scheduler' : {
            'id': 'ExponentialLR',
            'init': 1e-4,
            'gamma': 0.99
        }
    }
}