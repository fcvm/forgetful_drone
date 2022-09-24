import utils
from rospkg import RosPack; rp = RosPack ()
from pathlib import Path
import json
import re
import numpy as np
import matplotlib.pyplot as plt
import copy
from itertools import count

def save_json (d : dict or list, p : Path) -> None:
    with open (p, 'w') as f: f.write (json.dumps (d, sort_keys=True, indent=4))
def load_json (p : Path) -> dict or list:
    with open (p, 'r') as f: return json.load (f)

class TrainPlotter:
    _cnt = count (0)
    _colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
    
    def __init__(self, exp_id: str, lbl: str, reset_cnt: bool = False) -> None:
        
        self.cnt = next (self._cnt); print (self.cnt)
        if reset_cnt:
            self._cnt = count (0)
        
        self.lbl = lbl
        
        self.lr = []
        self.tl = []
        self.vl = []
        for rec in load_json (Path (rp.get_path ('forgetful_drones'))/'experiments'/exp_id/'output'/'train_records.json'):
            self.lr.append (rec ["learn_rate"])
            self.tl.append (rec ["train_loss"])
            self.vl.append (rec ["valid_loss"])
    
    def plotLR (self, ax) -> None:
        ax.plot (self.lr, label=self.lbl, linestyle='-.', color=self._colors [self.cnt])
    def plotTL (self, ax) -> None:
        ax.plot (self.tl, label=self.lbl, linestyle=':', color=self._colors [self.cnt])
    def plotVL (self, ax) -> None:
        ax.plot (self.vl, label=self.lbl, linestyle='-', color=self._colors [self.cnt])



if __name__ == '__main__':

    e1 = [
        TrainPlotter ('UTC_2022_09_14_07_18_53___E1R1_1GRUlayer', '1'),
    ]

    e2 = [
        TrainPlotter ('UTC_2022_09_14_07_18_53___E1R1_1GRUlayer', 'R1-1x64'),
        TrainPlotter ('UTC_2022_09_14_07_18_53___E1R1_2GRUlayer', 'R1-2x64'),
        TrainPlotter ('UTC_2022_09_14_07_18_53___E1R1_3GRUlayer', 'R1-3x64'),
        TrainPlotter ('UTC_2022_09_14_07_18_53___E1R1_5GRUlayer', 'R1-5x64'),
        TrainPlotter ('UTC_2022_09_14_07_18_53___E1R1_10GRUlayer', 'R1-10x64', reset_cnt=True),
    ]

    plots = [e2]

    plot_cnt = 0
    for plot in plots:
        fig, ax = plt.subplots (1, 1)
        
        for variant in plot:
            variant.plotTL (ax)
            variant.plotVL (ax)

        ax.set_xlabel ('Epochs [1]')
        ax.set_ylabel ('SmoothL1Loss')
        ax.legend () 
        #ax.legend (loc='lower left')
        ax.set_yscale('log')
        ax.grid ()
        utils.saveFig(__file__, suffix=f"_{plot_cnt}")
        plot_cnt += 1

