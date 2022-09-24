import utils
from rospkg import RosPack; rp = RosPack ()
from pathlib import Path
import json
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.axes import Axes
from matplotlib.lines import Line2D
import copy
from itertools import count
from typing import List

def save_json (d : dict or list, p : Path) -> None:
    with open (p, 'w') as f: f.write (json.dumps (d, sort_keys=True, indent=4))
def load_json (p : Path) -> dict or list:
    with open (p, 'r') as f: return json.load (f)


class VariantHandle:
    def plot (self, ax: Axes, color: str):
        pass

class TrainHandle (VariantHandle):

    def __init__(
        self, 
        exp_id: str, 
        lbl: str, 
        tl: bool = False,
        vl: bool = False, 
        lr: bool = False, 
    ) -> None:
        super ().__init__ ()
        
        self._lbl = lbl
        self._plottl = tl
        self._plotvl = vl
        self._plotlr = lr
    
        self._tl = []
        self._vl = []
        self._lr = []
        for rec in load_json (Path (rp.get_path ('forgetful_drones'))/'experiments'/exp_id/'output'/'train_records.json'):
            self._tl.append (rec ["train_loss"])
            self._vl.append (rec ["valid_loss"])
            self._lr.append (rec ["learn_rate"])
            
    
    
    def _plotTL (self, ax: Axes, color: str) -> None:
        #ax.plot (self.tl, label=self.lbl, linestyle=':', color=self._colors [self.cnt])
        ax.plot (self._tl, linestyle=(0, (1, 3)), color=color)
    def _plotVL (self, ax: Axes, color: str) -> None:
        #ax.plot (self.vl, label=self.lbl, linestyle='-', color=self._colors [self.cnt])
        ax.plot (self._vl, linestyle='-', color=color)
    def _plotLR (self, ax: Axes, color: str) -> None:
        #ax.plot (self.lr, label=self.lbl, linestyle='-.', color=self._colors [self.cnt])
        ax.plot (self._lr, linestyle='-.', color=color)
    def plot (self, ax, color) -> Line2D:
        if self._plottl: self._plotTL (ax, color)
        if self._plotvl: self._plotVL (ax, color)
        if self._plotlr: self._plotLR (ax, color)
        return mpatches.Patch (color=color, label=self._lbl)


class RacingHandle (VariantHandle):
    _scenes = list (range (5))
    _sites = list (range (3))
    _tracktypes = list (range (2))
    _trackgens = list (range (2))
    _trackdirs = list (range (2))
    _gatetypes = [1, 2]
    _maxspeeds = [4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
    _repetitions = list (range (10))

    def __init__(
        self,
        exp_id: str, 
        lbl: str,
        l_sc: List [int],
        l_si: List [int],
        l_tt: List [int],
        l_tg: List [int],
        l_td: List [int],
        l_gt: List [int],
        l: bool = False,
        v: bool = False, 
        t: bool = False, 
    ) -> None:
        super ().__init__ ()

        self._lbl = lbl
        self._l = l
        self._v = v
        self._t = t

        for selected, available, what in zip (
            [l_sc, l_si, l_tt, l_tg, l_td, l_gt],
            [self._scenes, self._sites, self._tracktypes, self._trackgens, self._trackdirs, self._gatetypes],
            ['scenes', 'sites', 'tracktypes', 'trackgens', 'trackdirs', 'gatetypes'],
        ):
            for index in selected:
                if index not in available:
                    raise ValueError (f"faulty index in selection for {what}")
        
        self._l_completions = [0] * len (self._maxspeeds)
        self._v_completions = [0] * len (self._maxspeeds)
        self._t_completions = [0] * len (self._maxspeeds)

        self._l_totals = [0] * len (self._maxspeeds)
        self._v_totals = [0] * len (self._maxspeeds)
        self._t_totals = [0] * len (self._maxspeeds)
        
        for rec in load_json (Path (rp.get_path ('forgetful_drones'))/'experiments'/exp_id/'racing'/'racing_records.json'):
            run_config = re.split('___|_', rec ['id'])
            index = self._maxspeeds.index (float (run_config [6]))

            self._t_totals [index] += 1
            if rec ['completed']: self._t_completions [index] += 1


            
            if int (run_config [0]) in l_sc and\
            int (run_config [1]) in l_si and\
            int (run_config [2]) in l_tt and\
            int (run_config [3]) in l_tg and\
            int (run_config [4]) in l_td and\
            int (run_config [5]) in l_gt:
                self._l_totals [index] += 1
                if rec ['completed']: self._l_completions [index] += 1
            else:
                self._v_totals [index] += 1
                if rec ['completed']: self._v_completions [index] += 1
        
        self._l_shares = []
        self._v_shares = []
        self._t_shares = []

        for x, y in zip (self._l_completions, self._l_totals):
            try: self._l_shares.append (100.0 * x / y)
            except: self._l_shares.append (None)
        for x, y in zip (self._v_completions, self._v_totals):
            try: self._v_shares.append (100.0 * x / y)
            except: self._v_shares.append (None)
        for x, y in zip (self._t_completions, self._t_totals):
            try: self._t_shares.append (100.0 * x / y)
            except: self._t_shares.append (None)


    def plot (self, ax, color) -> Line2D:
        if self._l: ax.plot (self._maxspeeds, self._l_shares, color=color, linestyle=':')
        if self._v: ax.plot (self._maxspeeds, self._v_shares, color=color, linestyle='-')
        if self._t: ax.plot (self._maxspeeds, self._t_shares, color=color, linestyle='-')
        return mpatches.Patch (color=color, label=self._lbl)








        
        

class ResultPlot:
    _colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']

    def __init__(
        self,
        name: str,
        handles: List [VariantHandle],
        xlabel: str = None,
        ylabel: str = None,
        yscale: str = None,
        legend_loc: str = None,
        grid: bool = True
    ) -> None:

        self._cnt = count (0)
        self._name = name
        self._handles = handles
        self._xlabel = xlabel
        self._ylabel = ylabel
        self._yscale = yscale
        self._grid = grid
        self._legend_loc = legend_loc
    
    def plot (self):
        fig, ax = plt.subplots (1, 1)
        
        handles = []
        for handle in self._handles:
            
            color = self._colors [next (self._cnt)]
            handles.append (handle.plot (ax, color))
            
        ax.set_xlabel (self._xlabel)
        ax.set_ylabel (self._ylabel)
        ax.legend (handles=handles, loc=self._legend_loc) 
        ax.set_yscale (self._yscale)
        ax.grid (self._grid)
        utils.saveFig(__file__, suffix=f"_{self._name}")


if __name__ == '__main__':

    for plot in [


        # EXPERIMENT 1

        ResultPlot (
            name='e1_learning_feedforward',
            handles=[
                TrainHandle ('UTC_2022_09_04_17_43_31', 'F1', tl=True, vl=True),
                TrainHandle ('UTC_2022_09_14_11_47_07', 'F2', tl=True, vl=True),
            ],
            xlabel='Epochs',
            ylabel='SmoothL1Loss',
            yscale='log',
            grid=True
        ),
        ResultPlot (
            name='e1_learning_recurrent',
            handles=[
                TrainHandle ('zUTC_2022_09_14_07_18_53', 'R1',     tl=True, vl=True),
                TrainHandle ('zUTC_2022_09_15_15_02_44', 'R2',     tl=True, vl=True),
                TrainHandle ('zUTC_2022_09_04_19_35_20___pretrainedCNN', 'R3',   tl=True, vl=True),
            ],
            xlabel='Epochs',
            ylabel='SmoothL1Loss',
            yscale='log',
            grid=True
        ),
        ResultPlot (
            name='e1_racing',
            handles=[
                RacingHandle ('UTC_2022_09_04_17_43_31',                    'F1', l_sc=[0], l_si=[0], l_tt=[0], l_tg=[1], l_td=[0], l_gt=[2], t=True),
                RacingHandle ('UTC_2022_09_14_11_47_07',                    'F2', l_sc=[0], l_si=[0], l_tt=[0], l_tg=[1], l_td=[0], l_gt=[2], t=True),
                RacingHandle ('zUTC_2022_09_14_07_18_53',                   'R1', l_sc=[0], l_si=[0], l_tt=[0], l_tg=[1], l_td=[0], l_gt=[2], t=True),
                RacingHandle ('zUTC_2022_09_15_15_02_44',                   'R2', l_sc=[0], l_si=[0], l_tt=[0], l_tg=[1], l_td=[0], l_gt=[2], t=True),
                RacingHandle ('zUTC_2022_09_04_19_35_20___pretrainedCNN',   'R3', l_sc=[0], l_si=[0], l_tt=[0], l_tg=[1], l_td=[0], l_gt=[2], t=True),
            ],
            xlabel='Max. Speed [m/s]',
            ylabel='Racetrack Completions [%]',
            yscale='linear',
            grid=True
        ),


        # EXPERIMENT 2

        ResultPlot (
            name='e2_learning',
            handles=[
                TrainHandle ('UTC_2022_09_14_07_18_53___E1R1_1GRUlayer', 'R1-1x64',     tl=True, vl=True),
                TrainHandle ('UTC_2022_09_14_07_18_53___E1R1_2GRUlayer', 'R1-2x64',     tl=True, vl=True),
                TrainHandle ('UTC_2022_09_14_07_18_53___E1R1_3GRUlayer', 'R1-3x64',     tl=True, vl=True),
                TrainHandle ('UTC_2022_09_14_07_18_53___E1R1_5GRUlayer', 'R1-5x64',     tl=True, vl=True),
                TrainHandle ('UTC_2022_09_14_07_18_53___E1R1_10GRUlayer', 'R1-10x64',   tl=True, vl=True),
            ],
            xlabel='Epochs',
            ylabel='SmoothL1Loss',
            yscale='log',
            grid=True
        ),
        ResultPlot (
            name='e2_racing',
            handles=[
                RacingHandle ('UTC_2022_09_14_07_18_53___E1R1_1GRUlayer', 'R1-1x64',   l_sc=[0], l_si=[0], l_tt=[0], l_tg=[1], l_td=[0], l_gt=[1, 2], t=True),
                RacingHandle ('UTC_2022_09_14_07_18_53___E1R1_2GRUlayer', 'R1-2x64',   l_sc=[0], l_si=[0], l_tt=[0], l_tg=[1], l_td=[0], l_gt=[1, 2], t=True),
                RacingHandle ('UTC_2022_09_14_07_18_53___E1R1_3GRUlayer', 'R1-3x64',   l_sc=[0], l_si=[0], l_tt=[0], l_tg=[1], l_td=[0], l_gt=[1, 2], t=True),
                RacingHandle ('UTC_2022_09_14_07_18_53___E1R1_5GRUlayer', 'R1-5x64',   l_sc=[0], l_si=[0], l_tt=[0], l_tg=[1], l_td=[0], l_gt=[1, 2], t=True),
                RacingHandle ('UTC_2022_09_14_07_18_53___E1R1_10GRUlayer', 'R1-10x64', l_sc=[0], l_si=[0], l_tt=[0], l_tg=[1], l_td=[0], l_gt=[1, 2], t=True),
            ],
            xlabel='Max. Speed [m/s]',
            ylabel='Racetrack Completions [%]',
            yscale='linear',
            grid=True
        ),


        # EXPERIMENT 3

        ResultPlot (
            name='e3_learning',
            handles=[
                TrainHandle ('UTC_2022_06_13_10_13_56', 'F', tl=True, vl=True),
                TrainHandle ('UTC_2022_07_01_07_27_56', 'R', tl=True, vl=True),
            ],
            xlabel='Epochs',
            ylabel='SmoothL1Loss',
            yscale='log',
            grid=True
        ),
        ResultPlot (
            name='e3_racing',
            handles=[
                RacingHandle ('UTC_2022_06_13_10_13_56', 'F',   l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
                RacingHandle ('UTC_2022_07_01_07_27_56', 'R',   l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
            ],
            xlabel='Max. Speed [m/s]',
            ylabel='Racetrack Completions [%]',
            yscale='linear',
            grid=True
        ),

        
        # EXPERIMENT 4

        ResultPlot (
            name='e4_learning',
            handles=[
                TrainHandle ('UTC_2022_07_01_07_27_56___SEQLEN_2', 'R-2*360x240',           tl=True, vl=True),
                TrainHandle ('UTC_2022_07_01_07_27_56', 'R-3*360x240',                      tl=True, vl=True),
                TrainHandle ('UTC_2022_07_01_07_27_56___SEQLEN_5', 'R-5*360x240',           tl=True, vl=True),
                TrainHandle ('UTC_2022_07_01_07_27_56___SEQLEN_10', 'R-10*360x240',         tl=True, vl=True),
                TrainHandle ('UTC_2022_07_01_07_27_56___SEQLEN_25', 'R-25*360x240',         tl=True, vl=True),
                TrainHandle ('UTC_2022_07_01_07_27_56___SEQLEN_2___RF3', 'R-2*240x160',     tl=True, vl=True),
                TrainHandle ('UTC_2022_07_01_07_27_56___SEQLEN_3___RF3', 'R-3*240x160',     tl=True, vl=True),
                TrainHandle ('UTC_2022_07_01_07_27_56___SEQLEN_5___RF3', 'R-5*240x160',     tl=True, vl=True),
                TrainHandle ('UTC_2022_07_01_07_27_56___SEQLEN_10___RF3', 'R-10*240x160',   tl=True, vl=True),
            ],
            xlabel='Epochs',
            ylabel='SmoothL1Loss',
            yscale='log',
            grid=True
        ),
        ResultPlot (
            name='e4_racing',
            handles=[
                RacingHandle ('UTC_2022_07_01_07_27_56___SEQLEN_2', 'R-2*360x240',           l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
                RacingHandle ('UTC_2022_07_01_07_27_56', 'R-3*360x240',                      l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
                RacingHandle ('UTC_2022_07_01_07_27_56___SEQLEN_5', 'R-5*360x240',           l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
                RacingHandle ('UTC_2022_07_01_07_27_56___SEQLEN_10', 'R-10*360x240',         l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
                RacingHandle ('UTC_2022_07_01_07_27_56___SEQLEN_25', 'R-25*360x240',         l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
                RacingHandle ('UTC_2022_07_01_07_27_56___SEQLEN_2___RF3', 'R-2*240x160',     l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
                RacingHandle ('UTC_2022_07_01_07_27_56___SEQLEN_3___RF3', 'R-3*240x160',     l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
                RacingHandle ('UTC_2022_07_01_07_27_56___SEQLEN_5___RF3', 'R-5*240x160',     l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
                RacingHandle ('UTC_2022_07_01_07_27_56___SEQLEN_10___RF3', 'R-10*240x160',   l_sc=[0, 1, 2, 3], l_si=[0, 1, 2], l_tt=[1], l_tg=[1], l_td=[0, 1], l_gt=[1, 2], l=True, v=True),
            ],
            xlabel='Max. Speed [m/s]',
            ylabel='Racetrack Completions [%]',
            yscale='linear',
            grid=True
        ),
        
    ]:
        plot.plot ()
