import utils
from rospkg import RosPack; rp = RosPack ()
from pathlib import Path
import json
import re
import numpy as np
import matplotlib.pyplot as plt
import copy


def save_json (d : dict or list, p : Path) -> None:
    with open (p, 'w') as f: f.write (json.dumps (d, sort_keys=True, indent=4))
def load_json (p : Path) -> dict or list:
    with open (p, 'r') as f: return json.load (f)


def selection (
    scenes=None, sites=None, 
    tracktypes=None, trackgens=None, trackdirs= None, 
    gatetypes=None, maxspeeds=None, repetitions=None
):
    _scenes = ['0', '1', '2', '3', '4']
    _sites = ['0', '1', '2']
    _tracktypes = ['0', '1']
    _trackgens = ['1']
    _trackdirs = ['0', '1']
    _gatetypes = ['1', '2']
    _maxspeeds = ['04.00', '05.00', '06.00', '07.00', '08.00', '09.00', '10.00']
    _repetitions = ['000', '001', '002', '003', '004', '005', '006', '007', '008', '009']

    if scenes is not None: _scenes = scenes
    if sites is not None: _sites = sites
    if tracktypes is not None: _tracktypes = tracktypes
    if trackgens is not None: _trackgens = trackgens
    if trackdirs is not None: _trackdirs = trackdirs
    if gatetypes is not None: _gatetypes = gatetypes
    if maxspeeds is not None: _maxspeeds = maxspeeds
    if repetitions is not None: _repetitions = repetitions

    selection = [
        _scenes,
        _sites,
        _tracktypes,
        _trackgens,
        _trackdirs,
        _gatetypes,
        _maxspeeds,
        _repetitions
    ]
    return selection


def filter_exp_by_sel (exp, index):
    THIS = copy.deepcopy (exp ['data'])
    THIS = THIS [index[0]]
    THIS = THIS [:, index[1]]
    THIS = THIS [:, :, index[2]]
    THIS = THIS [:, :, :, index[3]]
    THIS = THIS [:, :, :, :, index[4]]
    THIS = THIS [:, :, :, :, :, index[5]]
    THIS = THIS [:, :, :, :, :, :, index[6]]
    THIS = THIS [:, :, :, :, :, :, :, index[7]]
    return THIS

    


if __name__ == '__main__':
    
    #experiments = [
    #    'UTC_2022_06_13_10_13_56',
    #    'UTC_2022_06_22_21_15_38',
    #    'UTC_2022_07_01_07_27_56',
    #    'UTC_2022_07_01_07_27_56___SEQLEN_2',
    #    'UTC_2022_07_01_07_27_56___SEQLEN_5',
    #    'UTC_2022_07_01_07_27_56___SEQLEN_10',
    #    'UTC_2022_07_01_07_27_56___SEQLEN_25',
    #    'UTC_2022_07_01_07_27_56___SEQLEN_2___RF3',
    #    'UTC_2022_07_01_07_27_56___SEQLEN_3___RF3',
    #    'UTC_2022_07_01_07_27_56___SEQLEN_5___RF3',
    #    'UTC_2022_07_01_07_27_56___SEQLEN_10___RF3',
    #]

    i = 5
    experiments = [
        {
            'UTC_2022_06_13_10_13_56': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': 'non-seq, all',
                            'linestyle': '-'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': 'non-seq, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': 'non-seq, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    }
                ]
            },
            'UTC_2022_07_01_07_27_56': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': 'seq 3, all',
                            'linestyle': '-'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': 'seq 3, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': 'seq 3, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    },
                ]
                
            },
        },
        #1
        {
            'UTC_2022_07_01_07_27_56___SEQLEN_2': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': '360x240, 2',
                            'linestyle': '-'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    },
                ]
                
            },
            'UTC_2022_07_01_07_27_56': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': '360x240, 3',
                            'linestyle': '-'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_5': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:green',
                            'label': '360x240, 5',
                            'linestyle': '-'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_10': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:red',
                            'label': '360x240, 10',
                            'linestyle': '-'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_25': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:purple',
                            'label': '360x240, 25',
                            'linestyle': '-'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_2___RF3': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': '240x160, 2',
                            'linestyle': ':'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_3___RF3': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': '240x160, 3',
                            'linestyle': ':'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_5___RF3': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:green',
                            'label': '240x160, 5',
                            'linestyle': ':'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_10___RF3': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:red',
                            'label': '240x160, 10',
                            'linestyle': ':'
                        },
                        'scenes': None,
                        'tracktypes': ['1'],
                    }
                ]
            },
        },
        #2
        {
            'UTC_2022_07_01_07_27_56___SEQLEN_2': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': 'seq 2, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': 'seq 2, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    },
                ]
                
            },
            'UTC_2022_07_01_07_27_56': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': 'seq 3, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': 'seq 3, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_5': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:green',
                            'label': 'seq 5, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:green',
                            'label': 'seq 5, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_10': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:red',
                            'label': 'seq 10, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:red',
                            'label': 'seq 10, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_25': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:purple',
                            'label': 'seq 25, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:purple',
                            'label': 'seq 25, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    }
                ]
                
            },
        },
        #3
        {
            'UTC_2022_07_01_07_27_56___SEQLEN_2___RF3': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': 'seq 2, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': 'seq 2, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    },
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_3___RF3': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': 'seq 3, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': 'seq 3, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_5___RF3': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:green',
                            'label': 'seq 5, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:green',
                            'label': 'seq 5, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    }
                ]
                
            },
            'UTC_2022_07_01_07_27_56___SEQLEN_10___RF3': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:red',
                            'label': 'seq 10, train',
                            'linestyle': ':'
                        },
                        'scenes': ['0', '1', '2', '3'],
                        'tracktypes': ['1'],
                    },
                    {
                        'kwargs': {
                            'color': 'tab:red',
                            'label': 'seq 10, valid',
                            'linestyle': '--'
                        },
                        'scenes': ['4'],
                        'tracktypes': ['1'],
                    }
                ]
                
            },
        },
        #4
        {
            'UTC_2022_09_04_17_43_31': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': 'resnet8-fc3x256',
                            'linestyle': '-'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
            'UTC_2022_09_14_11_47_07': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': 'resnet14-fc3x256',
                            'linestyle': '-'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
            'zUTC_2022_09_14_07_18_53': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:green',
                            'label': 'resnet14-gru3x64',
                            'linestyle': '-'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
            'zUTC_2022_09_15_15_02_44': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:red',
                            'label': 'resnet14-gru3x64-opt',
                            'linestyle': '-'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
            'zUTC_2022_09_04_19_35_20___pretrainedCNN': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:purple',
                            'label': 'resnet14-gru3x64-pretrained',
                            'linestyle': '-'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
        },
        #5
        {
            'UTC_2022_09_14_07_18_53___E1R1_1GRUlayer': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:blue',
                            'label': 'resnet14-gru1x64',
                            'linestyle': '-'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
            'UTC_2022_09_14_07_18_53___E1R1_2GRUlayer': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:orange',
                            'label': 'resnet14-gru2x64',
                            'linestyle': '-'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
            'UTC_2022_09_14_07_18_53___E1R1_3GRUlayer': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:green',
                            'label': 'resnet14-gru3x64',
                            'linestyle': '-'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
            'zUTC_2022_09_14_07_18_53': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:green',
                            'label': 'resnet14-gru3x64.',
                            'linestyle': ':'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
            'UTC_2022_09_14_07_18_53___E1R1_5GRUlayer': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:red',
                            'label': 'resnet14-gru5x64',
                            'linestyle': '-'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
            'UTC_2022_09_14_07_18_53___E1R1_10GRUlayer': {
                'data': None,
                'plot_options': [
                    {
                        'kwargs': {
                            'color': 'tab:purple',
                            'label': 'resnet14-gru10x64',
                            'linestyle': '-'
                        },
                        'scenes': ['0'],
                        'sites': ['0'],
                        'tracktypes' : ['0'],
                        'trackdirs' : ['0'],
                    }
                ]
                
            },
        },
    ][i]

    






    dir = Path (rp.get_path ('forgetful_drones'))/'experiments'
    idx_maps = [{}, {}, {}, {}, {}, {}, {}, {}]

    for en in experiments.keys ():
        ep = dir/en

        for rec in load_json (ep/'racing/racing_records.json'):
            rec_vals = re.split('___|_', rec ['id'])
            for idx_map, rec_val in zip (idx_maps, rec_vals):
                try: idx_map [rec_val]
                except: 
                    idx = len (idx_map.keys ())
                    idx_map [rec_val] = idx

    size = tuple ([len (d.keys ()) for d in idx_maps])
    

    #print ('exps', json.dumps (exps, sort_keys=True, indent=4))
    #print ('idx_maps', json.dumps (idx_maps, sort_keys=True, indent=4))
    print ('size', json.dumps (size, sort_keys=True, indent=4))

    
    for en in experiments.keys ():
        ep = dir/en
        experiments [en] ['data'] = np.empty (size, dtype=int)
        experiments [en] ['data'] [:] = -999999999999
        for rec in load_json (ep/'racing/racing_records.json'):
            rec_vals = re.split('___|_', rec ['id'])
            index = []
            for idx_map, rec_val in zip (idx_maps, rec_vals):
                index.append (idx_map [rec_val])
            experiments [en] ['data'] [tuple(index)] = 1 if rec ['completed'] else 0



    fig, ax = plt.subplots (1, 1)
    maxSpeed = ['04.00', '05.00', '06.00', '07.00', '08.00', '09.00', '10.00']
    x = [float (x) for x in maxSpeed]

    for en in experiments.keys ():
        for po in experiments [en] ['plot_options']:
            succRate = []
            for v in maxSpeed:
                
                try: scenes = po ['scenes'] 
                except: scenes = None
                try: sites = po ['sites'] 
                except: sites = None
                try: tracktypes = po ['tracktypes'] 
                except: tracktypes = None
                try: trackgens = po ['trackgens'] 
                except: trackgens = None
                try: trackdirs = po ['trackdirs'] 
                except: trackdirs = None


                sels = selection (scenes=scenes, sites=sites, tracktypes=tracktypes, trackgens=trackgens, trackdirs=trackdirs, maxspeeds=[v])
                #try:
                #    sels = selection (scenes = po ['scenes'], tracktypes=['1'], maxspeeds=[v])
                #except:
                #    sels = selection (tracktypes=['1'], maxspeeds=[v])
            
                index = []
                for idx_map, sel in zip (idx_maps, sels):
                    index.append ([idx_map [s] for s in sel])

                filtered = filter_exp_by_sel (experiments [en], index)
                succRate.append (100 * float(filtered.sum ()) / filtered.size)
            
            
            ax.plot (x, succRate, **po ['kwargs'])
    
    ax.set_xlabel ('Max. Speed [m/s]')
    ax.set_ylabel ('Completion Rate [%]')
    ax.legend ()#(title='RGB Size; seq Len.')
    ax.grid ()
    utils.saveFig(__file__, suffix=f"_{i}")









                    







    

    
    

                

                


    #print (experiments)


