from rospkg import RosPack; rp = RosPack ()
from pathlib import Path
import json
import re
import numpy as np


def save_json (d : dict or list, p : Path) -> None:
    with open (p, 'w') as f: f.write (json.dumps (d, sort_keys=True, indent=4))
def load_json (p : Path) -> dict or list:
    with open (p, 'r') as f: return json.load (f)

if __name__ == '__main__':
    names = [
        'scene',
        'site',
        'track_type',
        'track_gen',
        'track_direc',
        'gate_type',
        'max_speed',
        'rep',
    ]
    dir = Path (rp.get_path ('forgetful_drones'))/'experiments'
    exps = {}
    idx_maps = [{}, {}, {}, {}, {}, {}, {}, {}]

    #exp_cnt = 0
    for e in dir.iterdir ():
        #print (exp_cnt); exp_cnt += 1
        if e.name.split ('_') [0] == 'UTC':
            exps [e.name] = {}

            
            #rec_cnt = 0
            for rec in load_json (e/'racing/racing_records.json'):
                #print (f"\t{rec_cnt}"); rec_cnt += 1
                rec_vals = re.split('___|_', rec ['id'])
                for idx_map, rec_val in zip (idx_maps, rec_vals):
                    try: idx_map [rec_val]
                    except: 
                        idx = len (idx_map.keys ())
                        idx_map [rec_val] = idx

    size = tuple ([len (d.keys ()) for d in idx_maps])
    

    #print ('exps', json.dumps (exps, sort_keys=True, indent=4))
    #print ('idx_maps', json.dumps (idx_maps, sort_keys=True, indent=4))
    #print ('size', json.dumps (size, sort_keys=True, indent=4))

    
    for e in dir.iterdir ():
        if e.name.split ('_') [0] == 'UTC':
            exps [e.name] = np.empty (size, dtype=int)
            exps [e.name] [:] = -999999999999

            for rec in load_json (e/'racing/racing_records.json'):
                rec_vals = re.split('___|_', rec ['id'])
                index = []
                for idx_map, rec_val in zip (idx_maps, rec_vals):
                    index.append (idx_map [rec_val])
                exps [e.name] [tuple(index)] = 1 if rec ['completed'] else 0




    #exp = next( iter (exps))
    exp = 'UTC_2022_07_01_07_27_56___SEQLEN_25'
    sels = [
        # scene
        [
            '0', 
            '1', 
            '2', 
            '3', 
            #'4'
        ],
        # site
        ['0', '1', '2'],
        # track_type ['0', '1']
    [
        #'0', 
        '1'
    ],
        # track_gen
        ['1'],
        # track_direc
        ['0', '1'],
        # gate_type
        ['1', '2'],
        # max_speed
        [
            '04.00', 
            '05.00', 
            '06.00', 
            '07.00', 
            '08.00', 
            #'09.00', 
            #'10.00'
        ],
        # rep
        ['000'],
    ]

    index = []
    for idx_map, sel in zip (idx_maps, sels):
        index.append ([idx_map [s] for s in sel])


    THIS = exps [exp] 
    THIS = THIS [index[0]]
    THIS = THIS [:, index[1]]
    THIS = THIS [:, :, index[2]]
    THIS = THIS [:, :, :, index[3]]
    THIS = THIS [:, :, :, :, index[4]]
    THIS = THIS [:, :, :, :, :, index[5]]
    THIS = THIS [:, :, :, :, :, :, index[6]]
    THIS = THIS [:, :, :, :, :, :, :, index[7]]

    print ('# Runs: ', THIS.size)
    print ('# Successful: ', THIS.sum ())
    print ('% Successful: ', float(THIS.sum ()) / THIS.size)
    pass









                    







    

    
    

                

                


    #print (experiments)


