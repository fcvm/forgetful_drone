#!/usr/bin/env python3
'#!/home/fm/env/pytorch/bin/python3'
DEBUG = False


import warnings
import ast



import numpy as np
import rospy
import rospkg; rospack = rospkg.RosPack()

import std_msgs.msg
import std_srvs.srv

import sensor_msgs.msg
import geometry_msgs.msg
import cv_bridge

from pathlib import Path
import json
import torch
from torch.optim import Optimizer
from torch.nn import Module

import torchinfo

import cv2
from tqdm import tqdm


import forgetful_drones.srv as fdsrv



from typing import List, Dict, Any
import pandas as pd
import copy

from forgetful_ann import ForgetfulANN
from forgetful_dataset import ForgetfulDataset

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick

import user_input



def subdirs (p: Path, sort : bool = True) -> List [Path]:
    l = [s for s in p.iterdir () if s.is_dir ()]
    return sorted (l) if sort else l

def ndirs (p: Path) -> int:
    return len (subdirs (p, sort=False))

def subfiles (p: Path, ext : None or str = None, sort : bool = True) -> List [Path]:
    if ext is None: l = [s for s in p.iterdir() if s.is_file()]
    else: l = [s for s in p.iterdir() if s.is_file() and s.suffix == ext]
    return sorted (l) if sort else l

def nfiles (p: Path, ext : None or str = None) -> int:
    return len (subfiles (p, ext=ext, sort=False))

def nlines (p: Path) -> int:
    return sum (1 for _ in open (p))

def isempty (p: Path) -> bool:
    if p.is_dir (): return 0 == ndirs (p) + nfiles (p)
    if p.is_file (): return 0 == nlines (p)
    else: raise AssertionError (f"Path \"{p}\" is not a directory or file")

def rmtree (p: Path) -> None:
    for s in p.iterdir (): s.unlink () if s.is_file () else rmtree (s)
    p.rmdir ()

def stempfx (path: Path, sep : str = '_') -> str:
    return path.stem.split (sep=sep) [0]

setattr (Path, 'subdirs', subdirs)
setattr (Path, 'subfiles', subfiles)
setattr (Path, 'ndirs', ndirs)
setattr (Path, 'nfiles', nfiles)
setattr (Path, 'nlines', nlines)
setattr (Path, 'isempty', isempty)
setattr (Path, 'rmtree', rmtree)
setattr (Path, "stempfx", stempfx)


def log (lvl: int, msg : str) -> None:
    if lvl == 0: msg = '\n' + msg
    elif lvl == 1: msg = '   - ' + msg
    elif lvl == 2: msg = '      - ' + msg
    elif lvl == 3: msg = '          - ' + msg
    else: raise ValueError ('Log level unknown')
    
    CBLUEBG = '\33[44m'
    print (CBLUEBG + msg + '\033[0m')

# utils
def repDict (d : Dict) -> None: 
    return json.dumps (d, sort_keys=True, indent=4)



def save_json (d : dict or list, p : Path) -> None:
    with open (p, 'w') as f: f.write (json.dumps (d, sort_keys=True, indent=4))
def load_json (p : Path) -> dict or list:
    with open (p, 'r') as f: return json.load (f)








'''class Recordings:
    def __init__ (self, p_pmn : Path, p_tmp : Path):
        self.p_pmn = p_pmn
        self.p_tmp = p_tmp
        self.data = {
            'run_id': [],
            'expert_intervention_share': [],
            'loss': {
                'train': [],
                'valid': [],
            },
            'learn_rate': [],
        }
        
    def file (self, mode : str , pmn : bool) -> Path:
        p = self.p_pmn if pmn else self.p_tmp
        if mode == 'save': save_json (self.data, p)
        elif mode == 'load': self.data = load_json (p)
        else: raise ValueError (f"Unknown mode: {mode}")
        return p

    def get_train_loss (self) -> List: return self.data ['loss'] ['train']
    def get_valid_loss (self) -> List: return self.data ['loss'] ['valid']
    def get_ei_share (self) -> List: return self.data ['expert_intervention_share']


    def include_run (self, id : str) -> bool: return id in self.data ['run_id']
    def num_runs (self) -> int: return len (self.data ['run_id'])
    def num_epochs (self, id : str) -> int:
        if not self.include_run (id):
            raise ValueError (f"Unrecorded run: {id}")
        idx = self.data ['run_id'].index (id)
        return len (self.data ['loss'] ['train'] [idx])



    def add_run (self, id: str) -> None:
        self.data ['run_id'].append (id)
        self.data ['loss'] ['train'].append ([])
        self.data ['loss'] ['valid'].append ([])
        self.data ['learn_rate'].append ([])

        if not int (id.split (sep='___') [0]) == self.num_runs () - 1:
            raise ValueError (f"Recorded run count mismatches with run ID")

    def add_eishare (self, x : float or None) -> None: self.data ['expert_intervention_share'].append (x)
    def add_tloss (self, x : float or None) -> None: self.data ['loss'] ['train'] [-1].append (x)
    def add_vloss (self, x : float or None) -> None: self.data ['loss'] ['valid'] [-1].append (x)
    def add_lrate (self, x : float or None) -> None: self.data ['learn_rate'] [-1].append (x)'''











    
    


class File:
    def __init__ (self, p_pmn : Path, p_tmp : Path):
        self._path_pmn = p_pmn      # permanent file path (to save final results)
        self._path_tmp = p_tmp      # temporary file path (to save temporary results)

    def _path (self, pmn : bool) -> Path:
        return self._path_pmn if pmn else self._path_tmp

    def save (self, pmn : bool) -> Path:
        raise NotImplementedError ()

    def load (self, pmn : bool) -> Path:
        raise NotImplementedError ()




class RunRecord (dict):
    id = 0
    
    def __init__ (self, id : str, eis : float):
        self ['id'] = id
        self ['eis'] = eis
        

class EpochRecord (dict):
    id = 1
    
    def __init__ (self, tl : float, vl : float, lr : float):
        self ['train_loss'] = tl
        self ['valid_loss'] = vl
        self ['learn_rate'] = lr
        

class Records:
    def __init__ (self, rec_id : int):
        self._data : List [Dict] = []         # list to store records sequentially
        self._rec_id = rec_id
    
    def add (self, rec : RunRecord or EpochRecord) -> None:
        if not self._rec_id == type(rec).id:
            raise ValueError ("Can only add records of same type")
        self._data.append (rec)
        
    def get_seq (self, key : str) -> List [Any]:
        return [rec [key] for rec in self._data]
    
    def num_records (self) -> int:
        return len (self._data)

    def recorded (self, id : str) -> bool:
        return id in [rec ['id'] for rec in self._data]


class RecordsFile (File, Records):
    def __init__ (self, p_pmn : Path, p_tmp : Path, rec_id : int):
        File.__init__ (self, p_pmn, p_tmp)
        Records.__init__ (self, rec_id)
       
    def save (self, pmn : bool) -> Path:
        save_json (self._data, self._path (pmn))
        return self._path (pmn)

    def load (self, pmn : bool) -> Path:
        self._data = load_json (self._path (pmn))
        return self._path (pmn)



class Config:
    def __init__ (self, new : bool):
        if new: self._mknew ()

    def _mknew (self) -> None:
        UI = Path (user_input.__file__)
        self._data = copy.deepcopy (user_input.user_input)
        
        # set cnf/data/raw/...
        self._data['data']['raw'] = {}
        self._data['data']['raw']['rgb'] = {}
        self._data['data']['raw']['rgb']['num_channels'] = 3
        self._data['data']['raw']['rgb']['width'] = rospy.get_param ('SIM_UNITY_DRONE_CAMERA_WIDTH')
        self._data['data']['raw']['rgb']['height'] = rospy.get_param ('SIM_UNITY_DRONE_CAMERA_HEIGHT')
        self._data['data']['raw']['nominal_rate'] = rospy.get_param ('MAIN_FREQ')
     
        # set cnf/data/processed/rgb/...
        rf = self._data['data']['processed']['rgb']['resize_factor']
        self._data['data']['processed']['rgb']['num_channels'] = self._data['data']['raw']['rgb']['num_channels']
        self._data['data']['processed']['rgb']['width'] = int (rf * self._data['data']['raw']['rgb']['width'])
        self._data['data']['processed']['rgb']['height'] = int (rf * self._data['data']['raw']['rgb']['height'])

        # set ann/cat/input_size
        inputsize = len(self._data['data']['input']['cat'])
        self._data['ann']['cat']['input_size'] = inputsize if (inputsize != 0) else None
        
        # set ann/head/output_size
        noutputs = len(self._data['data']['label'])
        if noutputs == 0: raise ValueError (f"File \"{UI}\", field \"data:label\": all lables disabled")
        self._data['ann']['head']['output_size'] = noutputs

        # set ann/gru/...
        seqlen = self._data['data']['sequential']['length']
        if seqlen < 1: raise ValueError (f"File \"{UI}\", field \"data:sequential:length\": not a positive integer")
        if seqlen == 1:
            self._data['ann']['gru']['num_layers'] = None
            self._data['ann']['gru']['hidden_size'] = None
            self._data['ann']['gru']['dropout'] = None

        # set ann/fc/...
        fc_nlayers = self._data['ann']['fc']['num_layers']
        if fc_nlayers is None:
            self._data['ann']['fc']['width'] = None
            self._data['ann']['fc']['activation_function_id'] = None
            self._data['ann']['fc']['dropout'] = None
        else:
            if fc_nlayers < 1: raise ValueError (f"File \"{UI}\", field \"fc:num_layers\": not a positive integer")

        self._init_members ()


    def _init_members (self) -> None:
        self.ann            : Dict  = self._data ['ann']
        self.opt_inp        : List  = self._data ['data'] ['input'] ['cat']
        self.cnn_inp        : List  = self._data ['data'] ['input'] ['cnn']
        self.lables         : List  = self._data ['data'] ['label']
        self.prc_rgb_W      : int   = self._data ['data'] ['processed'] ['rgb'] ['width']
        self.prc_rgb_H      : int   = self._data ['data'] ['processed'] ['rgb'] ['height']
        self.prc_rgb_N      : int   = self._data ['data'] ['processed'] ['rgb'] ['num_channels']
        self.seq_len        : int   = self._data ['data'] ['sequential'] ['length']
        self.seq_stp        : int   = self._data ['data'] ['sequential'] ['step']
        self.batch_size     : int   = self._data ['learn'] ['batch_size']
        self.loss_id        : str   = self._data ['learn'] ['loss'] ['id']
        self.lrsched_id     : str   = self._data ['learn'] ['lr_scheduler'] ['id']
        self.lrsched_gamma  : float = self._data ['learn'] ['lr_scheduler'] ['gamma']
        self.lrsched_init   : float = self._data ['learn'] ['lr_scheduler'] ['init']
        #self.num_epochs     : int   = self._data ['learn'] ['num_epochs'] 
        self.optim_id       : str   = self._data ['learn'] ['optimizer'] ['id']



class ConfigFile (File, Config):
    def __init__ (self, p_pmn : Path, p_tmp : Path, new : bool):
        File.__init__ (self, p_pmn, p_tmp)
        Config.__init__ (self, new)

    def save (self, pmn : bool) -> Path:
        save_json (self._data, self._path (pmn))
        return self._path (pmn)

    def load (self, pmn : bool) -> Path:
        self._data = load_json (self._path (pmn))
        self._init_members ()
        return self._path (pmn)




class DataFrameFile (File):
    def __init__ (self, p_pmn : Path, p_tmp : Path, seq_cols : List [str] or None):
        File.__init__ (self, p_pmn, p_tmp)
        self._data = pd.DataFrame ()
        self._seq_cols = seq_cols

    def save (self, pmn : bool) -> Path:
        self._data.to_csv (self._path (pmn), index=False)
        return self._path (pmn)

    def load (self, pmn : bool) -> Path:
        try: 
            self._data = pd.read_csv (self._path (pmn))
            if self._seq_cols is not None:
                self._load_seq ()
        except pd.errors.EmptyDataError: 
            self._data = pd.DataFrame ()
        return self._path (pmn)

    def _load_seq (self):
        for i in range (len (self._data)):
            for j in self._seq_cols:
                try: self._data.at [i, j] = json.loads (self._data.at [i, j])
                except json.decoder.JSONDecodeError:
                    self._data.at[i, j] = ast.literal_eval (self._data.at [i, j])

    def add (self, df : pd.DataFrame) -> None:
        self._data = pd.concat ([self._data, df], ignore_index=True)

    def num_samples (self) -> int:
        return len (self._data)

    def get (self) -> pd.DataFrame:
        return self._data



    
class CheckpointFile (File):
    def __init__ (self, p_pmn : Path, p_tmp : Path):
        File.__init__ (self, p_pmn, p_tmp)
        self._data = {}
    
    def save (self, pmn : bool) -> Path:
        torch.save (self._data, self._path (pmn))
        return self._path (pmn)

    def load (self, pmn : bool) -> Path:
        self._data = torch.load (self._path (pmn))
        return self._path (pmn)

    def set (self, ep : int, model : Module, optim : Optimizer, loss : Module) -> None:
        self._data = {
            'epoch': ep,
            'model_state_dict': model.state_dict (),
            'optimizer_state_dict': optim.state_dict (),
            'loss': loss
        }

    
    def get (self) -> Dict:
        return self._data

        






    







# CONSTANTS
TORCH_DEVICE = torch.device ('cuda' if torch.cuda.is_available() else 'cpu')
NCOLS = 0
BATCH_SIZE_INFER = 1










class ForgetfulBrain:

    def __init__ (self):
        
        # PATHS
        self._pkg = Path (rospack.get_path ('forgetful_drones'))
        
        # CONSTANTS
        self.RGB_EXT = '.jpg'
        self.RAW_COLS = [                                   # columns of raw data's data.txt files
                # whether a training sample is created
            'expert_intervened', 
                # inputs (without rgb)
            'rgb_dt', 'imu_dt', 
            'imu_linacc_x', 'imu_linacc_y', 'imu_linacc_z',
            'imu_angvel_x', 'imu_angvel_y', 'imu_angvel_z',
            'max_speed',
                # label: navigation decision 
            'exp_waypoint_x', 'exp_waypoint_y', 'exp_normspeed',
                # label: control command
            'ctrlcmd_bodyrates_x', 'ctrlcmd_bodyrates_y', 'ctrlcmd_bodyrates_z',
            'ctrlcmd_angacc_x', 'ctrlcmd_angacc_y', 'ctrlcmd_angacc_z',
            'ctrlcmd_collthrust'
        ]
        self.RGB_COLS = ['rgb_fpath']                       # columns that are input to the CNN
        self.IMD_COLS = [                                   # columns of intermediate data (->insert RGB_COLS in RAW_COLS)
            *self.RAW_COLS [:2],
            *self.RGB_COLS,
            *self.RAW_COLS [2:],
        ]
        self.PRC_COLS = self.IMD_COLS [1:]                  # columns of processed data (->cut 'expert_intervened' column of IMD_COLS)
        self.PRC_SEQ_COLS = self.PRC_COLS [:10]             # columns of processed data that contain sequences (->no labels, only inputs)
        self.OPT_COLS = self.RAW_COLS [1: 10]               # columns that are input to the CAT


        # ALLOCATIONS
        self.expID : str or None = None                     # ID of experiment
        self.runID : str or None = None                     # ID of run
        
        # FILES
        self._cnf : Config or None = None

        self.model : ForgetfulANN or None = None            # model
        self.optim : Optimizer or None = None   # optimizer
        self.lrSched : torch.optim.lr_scheduler._LRScheduler or None = None # learning rate scheduler
        self.loss : Module or None = None          # loss
        self.optMask : List [int] or None = None            # mask to filter only selected optional input


        # ROS
        #  - service servers
        self.srv_initExp = rospy.Service ("brain/init_experiment", fdsrv.String, self.cb_initExp)
        self.srv_buildRun = rospy.Service ("brain/build_run", fdsrv.String, self.cb_buildRun)
        self.srv_startTrain = rospy.Service ("brain/start_training", fdsrv.Int, self.cb_startTrain)
        self.srv_startInfer = rospy.Service ("brain/start_inference", fdsrv.Float, self.cb_startInfer)
    
        
    ##############################################################################################################
    ##############################################################################################################

    # ROS CALLBACKS

    #  - service servers

    def cb_initExp (self, req : fdsrv.StringRequest) -> fdsrv.StringResponse: 
        self.initExp (id=req.data)
        return fdsrv.StringResponse ()
    
    def cb_buildRun (self, req : fdsrv.StringRequest) -> fdsrv.StringResponse: 
        self.buildRun (id=req.data)
        return fdsrv.StringResponse ()

    def cb_startTrain (self, req : fdsrv.IntRequest) -> fdsrv.IntResponse:
        self.startTrain (num_epochs=req.data, reset_lrsched=True)
        return fdsrv.IntResponse ()

    def cb_startInfer (self, req : fdsrv.FloatRequest) -> fdsrv.FloatResponse: 
        self.startInfer (max_speed=req.data)
        return fdsrv.FloatResponse ()

    #  - during inference

    def cb_RGB (self, msg : sensor_msgs.msg.Image) -> None: 
        self.rgb_mgs = msg
    
    def cb_IMU (self, msg : sensor_msgs.msg.Imu) -> None: 
        self.imu_msg = msg
    
    def cb_INF (self, _ : std_msgs.msg.Empty) -> None: 
        self.INF()


    ##############################################################################################################
    ##############################################################################################################

    # ASSERT

    def asrExpInit (self) -> None: 
        if self.expID is None: raise ValueError (f"Experiment should be initialized")

    def asrPath (self, p : Path or None, isdir : bool, exists : bool) -> None:
        if p is None: raise ValueError (f"Path should be initialized")
        if isdir:
            if exists:
                if not p.is_dir(): raise FileNotFoundError (str (self._rp (p)))
            else:
                if p.is_dir(): raise FileExistsError (str (self._rp (p)))
        else:
            if exists:
                if not p.is_file(): raise FileNotFoundError (str (self._rp (p)))
            else:
                if p.is_file(): raise FileExistsError (str (self._rp (p)))

    ##############################################################################################################
    ##############################################################################################################

    # PATHS
    
    def _exp_                   (self)          -> Path: self.asrExpInit (); return self._pkg/'experiments'/self.expID
    
    def _exp_cnf_               (self)          -> Path: return self._exp_()/'config'
    def _exp_cnf_YML            (self)          -> Path: return self._exp_cnf_()/'forgetful_drone.yaml'
    def _exp_cnf_CNF            (self)          -> Path: return self._exp_cnf_()/'config.json'
    
    def _exp_dat_               (self)          -> Path: return self._exp_()/'data'
    def _exp_dat_raw_           (self)          -> Path: return self._exp_dat_()/'raw'
    def _exp_dat_raw_rid_       (self, rid:str) -> Path: return self._exp_dat_raw_()/rid
    def _exp_dat_raw_rid_rgb_   (self, rid:str) -> Path: return self._exp_dat_raw_rid_(rid)/'rgb'
    def _exp_dat_raw_rid_TXT    (self, rid:str) -> Path: return self._exp_dat_raw_rid_(rid)/'data.txt'
    
    def _exp_dat_idm_           (self)          -> Path: return self._exp_dat_()/'intermediate'
    def _exp_dat_imd_RID        (self, rid:str) -> Path: return self._exp_dat_idm_()/f"{rid}.csv"
    
    def _exp_dat_prc_           (self)          -> Path: return self._exp_dat_()/'processed'
    def _exp_dat_prc_PRC        (self)          -> Path: return self._exp_dat_prc_()/f"processed.csv"
    def _exp_dat_prc_rid_       (self, rid:str) -> Path: return self._exp_dat_prc_()/rid
    def _exp_dat_prc_rid_rgb_   (self, rid:str) -> Path: return self._exp_dat_prc_rid_(rid)/'rgb'
    def _exp_dat_prc_rid_CSV    (self, rid:str) -> Path: return self._exp_dat_prc_rid_(rid)/'data.csv'
    
    def _exp_out_               (self)          -> Path: return self._exp_()/'output'
    def _exp_out_BRC            (self)          -> Path: return self._exp_out_()/'build_records.json'
    def _exp_out_TRC            (self)          -> Path: return self._exp_out_()/'train_records.json'
    def _exp_out_CPT            (self)          -> Path: return self._exp_out_()/'checkpoint.pt'
    def _exp_out_ANT            (self)          -> Path: return self._exp_out_()/'model_scripted_with_annotation.pt'
    def _exp_out_TRA            (self)          -> Path: return self._exp_out_()/'model_scripted_with_tracing.pt'
    
    def _exp_out_plt_           (self)          -> Path: return self._exp_out_()/'plot'
    def _exp_out_plt_LSS        (self)          -> Path: return self._exp_out_plt_()/'loss.pdf'
    def _exp_out_plt_EIS        (self)          -> Path: return self._exp_out_plt_()/'expert_intervention_share.pdf'

    def _exp_out_tmp_           (self)          -> Path: return self._exp_out_()/'tmp'
    def _exp_out_tmp_RBI        (self, rid:str) -> Path: return self._exp_out_tmp_()/f'_{rid}_build_incomplete.info'
    def _exp_out_tmp_PRC        (self)          -> Path: return self._exp_out_tmp_()/self._exp_dat_prc_PRC().name
    def _exp_out_tmp_BRC        (self)          -> Path: return self._exp_out_tmp_()/self._exp_out_BRC().name
    def _exp_out_tmp_TRC        (self)          -> Path: return self._exp_out_tmp_()/self._exp_out_TRC().name
    def _exp_out_tmp_CPT        (self)          -> Path: return self._exp_out_tmp_()/self._exp_out_CPT().name
    def _exp_out_tmp_ANT        (self)          -> Path: return self._exp_out_tmp_()/self._exp_out_ANT().name
    def _exp_out_tmp_TRA        (self)          -> Path: return self._exp_out_tmp_()/self._exp_out_TRA().name
    def _exp_out_tmp_LSS        (self)          -> Path: return self._exp_out_tmp_()/self._exp_out_plt_LSS().name

    # - utils

    def _rp (self, p:Path)  -> Path: 
        return p.relative_to (self._pkg/'experiments')
    
    def _newExp_ (self, isdir : bool, exist : bool) -> List [Path]:
        if isdir: return [
                self._exp_ (),
                self._exp_cnf_ (),
                self._exp_dat_ (),
                self._exp_dat_raw_ (),
            ] if exist else [
                self._exp_dat_idm_ (),
                self._exp_dat_prc_ (),
                self._exp_out_ (),
                self._exp_out_plt_ (),
                self._exp_out_tmp_ (),
            ]
        else: return [
                self._exp_cnf_YML (),
            ] if exist else [
                self._exp_cnf_CNF (),
                self._exp_dat_prc_PRC (),
                self._exp_out_BRC (),
                self._exp_out_TRC (),
                self._exp_out_CPT (),
                #self._exp_out_ANT (),
                #self._exp_out_TRA (),
                #self._exp_out_plt_LSS (),
                #self._exp_out_plt_EIS (),
            ]


    def _exExp_ (self, isdir : bool) -> List [Path]:
        return [
            *self._newExp_ (isdir=isdir, exist=True),
            *self._newExp_ (isdir=isdir, exist=False),
        ]


    def _newRun_ (self, rid : str, isdir : bool, exist : bool) -> List [Path]:
        if isdir: return [
                self._exp_dat_raw_rid_ (rid),
                self._exp_dat_raw_rid_rgb_ (rid),
            ] if exist else [
                self._exp_dat_prc_rid_ (rid),
                self._exp_dat_prc_rid_rgb_ (rid),
            ]
        else: return [
                self._exp_dat_raw_rid_TXT (rid),
            ] if exist else [
                self._exp_dat_imd_RID (rid),
                self._exp_dat_prc_rid_CSV (rid),
            ]

        

    
    
    
    
    ##############################################################################################################
    ##############################################################################################################

    def log (self, msg : str, lvl : int or None) -> None:
        if lvl is None: return
        
        if lvl == 0: msg = '\n' + msg
        elif lvl == 1: msg = '   - ' + msg
        elif lvl == 2: msg = '      - ' + msg
        elif lvl == 3: msg = '          - ' + msg
        else: raise ValueError ('Log level unknown')

        CBLUEBG = '\33[44m'
        print (CBLUEBG + msg + '\033[0m')

    def log_load (self, p : Path, lvl : int or None) -> None: self.log (f"Loaded \"{self._rp (p)}\"", lvl)
    def log_save (self, p : Path, lvl : int or None) -> None: self.log (f"Saved \"{self._rp (p)}\"", lvl)
    def log_create (self, p : Path, lvl : int or None) -> None: self.log (f"Created \"{self._rp (p)}\"", lvl)
    def log_remove (self, p : Path, lvl : int or None) -> None: self.log (f"Removed \"{self._rp (p)}\"", lvl)
    

    ##############################################################################################################
    ##############################################################################################################

    # SAVE & LOAD
    def loadDF (self, p : Path, seq : bool, log_lvl : int or None = None) -> pd.DataFrame:
        try: 
            df = pd.read_csv (p)
            if seq:
                for i in range (len (df)):
                    for j in self.PRC_SEQ_COLS:
                        try: df.at [i, j] = json.loads (df.at [i, j])
                        except json.decoder.JSONDecodeError:
                            df.at[i, j] = ast.literal_eval (df.at [i, j])
        except pd.errors.EmptyDataError: 
            df = pd.DataFrame ()

        self.log_load (p, log_lvl)
        return df

    
    def saveScriptModule (self, ant : bool, pmn : bool, log_lvl: int or None = None) -> None:
        if ant:
            p = self._exp_out_ANT () if  pmn else self._exp_out_tmp_ANT ()
            self.model.exportAsAnnotatedTorchScriptModule (p)
        else:
            p = self._exp_out_TRA () if pmn else self._exp_out_tmp_TRA ()
            batch = next (iter (self.dataloader))
            self.model.exportAsTracedTorchScriptModule (
                batch ['input'] ['cnn'].to (TORCH_DEVICE),
                batch ['input'] ['cat'].to (TORCH_DEVICE),
                p
            )
        self.log_save (p, log_lvl)

    def saveLossPlt (self, pmn : bool, log_lvl : int or None = None) -> None:

        # PLOT CONFIG
        plt.rcParams['text.usetex'] = True
        clr = 'tab:orange'
        
        # CREATE FIG & AX
        with warnings.catch_warnings():
            warnings.simplefilter ("ignore")
            fig, ax = plt.subplots (figsize=(10, 7))
        
        # PLOT TRAIN LOSS
        ep_idx = 0
        y = self._trnRec.get_seq ('train_loss')
        x = list (range (self._trnRec.num_records ()))
        lbl = 'Training'
        plt.plot (x, y, color=clr, linestyle=':', label=lbl)

        # PLOT VALID LOSS
        y = self._trnRec.get_seq ('valid_loss')
        lbl = 'Validation'
        plt.plot (x, y, color=clr, linestyle='-', label=lbl)
        
        # AX CONFIG
        ax.set_xlabel ('Epochs')
        ax.set_ylabel (self._cnf.loss_id, color=clr)
        ax.tick_params (axis='y', labelcolor=clr)
        #ax.yaxis.set_major_formatter (mtick.FormatStrFormatter ('%.3e'))
        ax.legend (loc='lower left')
        ax.set_yscale('log')

        # SAVE FIG
        fig.tight_layout()
        p = self._exp_out_plt_LSS () if pmn else self._exp_out_tmp_LSS ()
        plt.savefig (p)

        # LOG
        self.log_save (p, log_lvl)

    def saveEISPlot (self, log_lvl : int or None = None) -> None: 
        
        # PLOT CONFIG
        plt.rcParams['text.usetex'] = True
        clr = 'tab:green'

        # CREATE FIG & AX
        with warnings.catch_warnings():
            warnings.simplefilter ("ignore")
            fig, ax = plt.subplots (figsize=(10, 7))

        # PLOT
        plt.plot (self._bldRec.get_seq ("eis"), color=clr)
        
        # AX CONFIG
        ax.set_xlabel ('Runs')
        ax.set_ylabel ('Expert Intervention Share', color=clr)
        ax.tick_params(axis='y', labelcolor=clr)

        # SAVE FIG
        fig.tight_layout()
        p = self._exp_out_plt_EIS ()
        plt.savefig (p)

        # LOG
        self.log_save (p, log_lvl)


    ##############################################################################################################
    ##############################################################################################################

    # INIT EXPERIMENT
    
    def initExp (self, id : str) -> None:
        """
        1) Set experiment ID
        2) If new experiment:
            1. Assert tree and create sub-directories
            2. Create new config.json from user_input.py
            3. Create empty processed.csv
            4. Create empty build_records.json
            5. Create empty train_records.json
            6. Init model, optimizer, etc.
            7. Create checkpoint.pt
        3) If existing experiment:
            1. Assert tree
            2. Load config.json
            3. Load processed.csv
            4. Load build_records.json
            5. Load train_records.json
            6. Init model, optimizer, etc.
            7. Load checkpoint.pt

        """

        self.log ('INIT EXPERIMENT', 0)
        
        self.expID = id
        self.log (f"ID: {self.expID}", 1)

        if not self._exp_cnf_CNF ().is_file ():
            self.createExp ()
        else:
            self.loadExp ()

        
    def createExp (self) -> None:
        self.log (f"Create new", 1)

        LOGLVL = 2
        PMN = True
        
        # ASSERT DIRS/FILES DO/DO NOT EXIST AND CREATE NON EXISTING DIRS
        for exist in [True, False]:
            for isdir in [True, False]:
                for path in self._newExp_ (isdir=isdir, exist=exist):
                    self.asrPath (path, isdir=isdir, exists=exist)
                    if exist is False and isdir is True:
                        path.mkdir (parents=True, exist_ok=False)
                        self.log_create (path, LOGLVL)

        # CREATE MISSING FILES
        self._cnf = ConfigFile (                    # configuration File
            self._exp_cnf_CNF (), 
            None,
            new=True
        )
        self.log_create (
            self._cnf.save (pmn=PMN), LOGLVL)
        
        self._prc = DataFrameFile (                 # processed data file
            self._exp_dat_prc_PRC (), 
            self._exp_out_tmp_PRC (), 
            self.PRC_SEQ_COLS
        )
        self.log_create (
            self._prc.save (pmn=PMN), LOGLVL)

        self._bldRec = RecordsFile (                # data building recordings
            self._exp_out_BRC (), 
            self._exp_out_tmp_BRC (),
            RunRecord.id
        )
        self.log_create (
            self._bldRec.save (pmn=PMN), LOGLVL)

        self._trnRec = RecordsFile (                # training recordings
            self._exp_out_TRC (), 
            self._exp_out_tmp_TRC (),
            EpochRecord.id
        )
        self.log_create (
            self._trnRec.save (pmn=PMN), LOGLVL)

        # INIT MODEL ETC
        self.initModelEtc ()

        # SAVE CHECKPOINT
        self._cpt = CheckpointFile (
            self._exp_out_CPT (),
            self._exp_out_tmp_CPT ()
        )
        self._cpt.set (
            0,
            self.model,
            self.optim,
            self.loss
        )
        self.log_create (
            self._cpt.save (pmn=PMN), LOGLVL)


    def loadExp (self) -> None:
        self.log (f"Load existing", 1)
        
        LOGLVL = 2
        PMN = True
        
        # ASSERT DIRS/FILES  EXIST
        for isdir in [True, False]:
            for path in self._exExp_ (isdir=isdir):
                self.asrPath (path, isdir=isdir, exists=True)

        # LOAD FILES
        self._cnf = ConfigFile (                    # configuration File
            self._exp_cnf_CNF (), 
            None,
            new=False
        )
        self.log_load (
            self._cnf.load (pmn=PMN), LOGLVL)
        
        self._prc = DataFrameFile (                 # processed data file
            self._exp_dat_prc_PRC (), 
            self._exp_out_tmp_PRC (), 
            self.PRC_SEQ_COLS
        )
        self.log_load (
            self._prc.load (pmn=PMN), LOGLVL)

        self._bldRec = RecordsFile (                # data building recordings
            self._exp_out_BRC (), 
            self._exp_out_tmp_BRC (),
            RunRecord.id
        )
        self.log_load (
            self._bldRec.load (pmn=PMN), LOGLVL)

        self._trnRec = RecordsFile (                # training recordings
            self._exp_out_TRC (), 
            self._exp_out_tmp_TRC (),
            EpochRecord.id
        )
        self.log_load (
            self._trnRec.load (pmn=PMN), LOGLVL)

        # INIT MODEL ETC
        self.initModelEtc ()

        # LOAD CHECKPOINT
        self._cpt = CheckpointFile (
            self._exp_out_CPT (),
            self._exp_out_tmp_CPT ()
        )
        self.log_load (
            self._cpt.load (pmn=PMN), LOGLVL)
        self.log (f"last epoch idx: {self._cpt.get () ['epoch']}", LOGLVL + 1)
        self.loadCheckpoint (reset_lrsched=False)
        

    
    def initModelEtc (self) -> None:
        self.model = self.initModel ()
        self.optim = self.initOptimizer ()
        self.lrSched = self.initLRScheduler ()
        self.loss = self.initLoss ()
        self.optMask = self.initOptMask ()
    
    def loadCheckpoint (self, reset_lrsched : bool) -> None:
        self.model.load_state_dict (self._cpt.get () ['model_state_dict'])
        self.optim.load_state_dict (self._cpt.get () ['optimizer_state_dict'])
        if reset_lrsched:
            for _ in self.optim.param_groups: _['lr'] = self._cnf.lrsched_init
        self.loss = self._cpt.get () ['loss']

        

    def initModel (self, log_on : bool = True) -> ForgetfulANN:
        return ForgetfulANN (self._cnf.ann, log_on).to (TORCH_DEVICE)

    def initOptimizer (self) -> Optimizer:
        return {
            'Adam': torch.optim.Adam    (self.model.parameters (), lr=self._cnf.lrsched_init),
            'SGD':  torch.optim.SGD     (self.model.parameters (), lr=self._cnf.lrsched_init),
        } [self._cnf.optim_id]

    def initLRScheduler (self) -> torch.optim.lr_scheduler._LRScheduler:
        return {
            'ExponentialLR': torch.optim.lr_scheduler.ExponentialLR (self.optim, gamma=self._cnf.lrsched_gamma)
        } [self._cnf.lrsched_id]

    def initLoss (self) -> Module:
        return {
            'SmoothL1Loss': torch.nn.SmoothL1Loss (),
            'MSELoss':      torch.nn.MSELoss ()
        } [self._cnf.loss_id]
    
    def initOptMask (self) -> List [int]:
        mask = []
        for i in self._cnf.opt_inp:
            for idx, j in enumerate (self.OPT_COLS):
                if i == j: 
                    mask.append(idx)
                    break
        return mask

    ##############################################################################################################
    ##############################################################################################################

    def getModelSummary (self) -> None:
        C = self._cnf.prc_rgb_N
        H = self._cnf.prc_rgb_H
        W = self._cnf.prc_rgb_W
        x_shape = (1, 1, C, H, W)
        o_shape = (1, 1, len (self._cnf.opt_inp))
        h_shape = tuple(self.model.getZeroInitializedHiddenState (1, TORCH_DEVICE).shape)
        torchinfo.summary (self.model, input_size=(x_shape, o_shape, h_shape))

    ##############################################################################################################
    ##############################################################################################################


    # BUILD RUN

    def buildRun (self, id : str) -> None:
        """
        1) Sets run ID
        2) If run is built: Do nothing
        3) If run not built:
            1. If run incompletely built: Remove incomplete data
            2. Assert tree and create sub-directories
            3. Build/save run's intermediate/*.csv
            4. Build/save run's processed/*/data.csv
            5. Build/save experiment's processed.csv
            6. Add run and save build recordings
        """
        self.asrExpInit ()
        self.switchInferTopics (register=False)

        self.log ('BUILD RUN', 0)
        
        self.runID = id
        self.log (f"ID: {self.runID}", 1)

        if self._bldRec.recorded (self.runID):
            self.log (f"Found complete build", 1)
        else:
            if self._exp_out_tmp_RBI (self.runID).is_file ():   # build of run is incomplete (e.g., it was aborted)
                self.buildRun__clearIncomplete ()
            self.buildRun__build ()

    
    def buildRun__clearIncomplete (self) -> bool:
        self.log (f"Found incomplete build", 1)

        for p in self._newRun_ (self.runID, isdir=True, exist=False):
            if p.is_dir ():
                p.rmtree ()
                self.log_remove (p, 2)
        
        for p in self._newRun_ (self.runID, isdir=False, exist=False):
            if p.is_file(): 
                p.unlink ()
                self.log_remove (p, 2)

    def buildRun__build (self) -> None:

        # CREATE TMP BUILD FILE
        self._exp_out_tmp_RBI (self.runID).touch ()

        # ASSERT DIRS/FILES DO/DO NOT EXIST AND CREATE NON EXISTING DIRS
        for exist in [True, False]:
            for isdir in [True, False]:
                for path in self._newRun_ (self.runID, isdir=isdir, exist=exist):
                    self.asrPath (path, isdir=isdir, exists=exist)
                    if exist is False and isdir is True:
                        path.mkdir (parents=True, exist_ok=False)
                        self.log_create (path, 2)

        
        # BUILD DATA
        self.log ("Build intermediate data of run", 1)
        dfIMD = self.buildRun__Imd (self.runID, log_lvl=2)
        
        self.log ('Build sequential data of run', 1)
        dfSEQ, eis = self.buildRun__Seq (self.runID, dfIMD=dfIMD, log_lvl=2)

        # PROCESSED DATA
        self.log ('Update processed data of experiment', 1)
        self.log_load (self._prc.load (pmn=True), 2)
        prv_ns = self._prc.num_samples ()
        self._prc.add (dfSEQ)
        self.log (f"# samples: {prv_ns} -> {self._prc.num_samples ()}", 2)
        self.log_save (self._prc.save (pmn=False), 2)

        # RECORDINGS
        self.log ('Update build records', 1)
        self.log_load (self._bldRec.load (pmn=True), 2)
        self._bldRec.add (RunRecord (self.runID, eis))
        self.log ('Added run record', 2)
        self.log_save (self._bldRec.save (pmn=False), 2)

        # FINAL BUILD SAVINGS & REMOVE TMP BUILD FILE
        self._exp_out_tmp_RBI (self.runID).unlink ()

        self.log ('Save permanently', 1)
        self.log_save (self._prc.save (pmn=True), 1)
        self.log_save (self._bldRec.save (pmn=True), 1)
        self.saveEISPlot (log_lvl=1)
        

        


    def buildRun__Imd (self, run_id : str, log_lvl : int or None = None) -> pd.DataFrame:

        # PATHS
        RGB = self._exp_dat_raw_rid_rgb_ (run_id)     # to the run's dir of raw rgbs (source)
        TXT = self._exp_dat_raw_rid_TXT (run_id)     # to the run's raw txt file (source)
        CSV = self._exp_dat_imd_RID (run_id)      # to the run's intermediate csv file (destination)
        
        # DATA FRAMES
        dfRGB = pd.DataFrame (
            RGB.subfiles (ext=self.RGB_EXT, sort=True),   # sorted list of paths to .jpg files in RGB
            columns=self.RGB_COLS
        )
        dfTXT = pd.read_csv (TXT)
        dfTXT [self.RAW_COLS [9]] = dfTXT [self.RAW_COLS [9]].astype (float)  # max_speed column is saved as int

        # ASSERT
        if list (dfTXT.columns) != self.RAW_COLS:
            raise ValueError (f"Corrupted columns in file \"{self._rp (TXT)}\"")
        if len (dfRGB) != len (dfTXT):
            raise ValueError (f"Mismatching # samples in file \"{self._rp (TXT)}\" and directory \"{self._rp (RGB)}\"")

        # DATAFRAME: intermediate
        dfIMD = pd.concat ([dfRGB, dfTXT], axis=1) [self.IMD_COLS]
        dfIMD.to_csv (CSV, index=False)

        # LOGGING
        self.log (f"# samples: {len (dfIMD)}", log_lvl)
        self.log_save (CSV, log_lvl)

        return dfIMD



    
    def buildRun__Seq (self, run_id : str, dfIMD : pd.DataFrame or None = None, log_lvl : int or None = None) -> None:
        
        # PATHS
        RGB = self._exp_dat_prc_rid_rgb_ (run_id)     # to the run's dir of processed rgbs (destination)
        SEQ = self._exp_dat_prc_rid_CSV (run_id)     # to the run's processed csv file (destination)

        # DATA FRAMES
        if dfIMD is None: dfIMD = pd.read_csv (self._exp_dat_imd_RID (run_id))
        dfSEQ = pd.DataFrame ()

        # DATA SEQUENCING
        with tqdm( range( len(dfIMD)), desc='|  Data Sequencing', leave=False, position=1, ncols=NCOLS) as pbar:
            ei_cnt = 0                                                  # count of expert interventions
            ss_cnt = 1                                                  # count of sequential steps
            
            for i in pbar:                                              # (for every sample in df_imd)
                ss_cnt -= 1
                if not dfIMD.iloc [i] ['expert_intervened']: continue  # skip this sample, if the expert did not intervene
                
                ei_cnt += 1

                if ss_cnt > 1: continue                                 # sequential step implementation

                rgb_dt = []; rgb_fp = []; imu_dt = []                   # Init sequences of inputs
                imu_lx = []; imu_ly = []; imu_lz = []
                imu_ax = []; imu_ay = []; imu_az = []
                mspeed = []
                
                # Fill the sequences with the previous samples (according to the sequence length)
                for j in range (i + 1 - self._cnf.seq_len, i + 1):
                    if j < 0: break                                     # Skip, if there's not enough previous samples

                    rgb_dt.append (dfIMD.iloc [j] ['rgb_dt'])
                    rgb_fp.append (dfIMD.iloc [j] ['rgb_fpath'])
                    imu_dt.append (dfIMD.iloc [j] ['imu_dt'])
                    imu_lx.append (dfIMD.iloc [j] ['imu_linacc_x'])
                    imu_ly.append (dfIMD.iloc [j] ['imu_linacc_y'])
                    imu_lz.append (dfIMD.iloc [j] ['imu_linacc_z'])
                    imu_ax.append (dfIMD.iloc [j] ['imu_angvel_x'])
                    imu_ay.append (dfIMD.iloc [j] ['imu_angvel_y'])
                    imu_az.append (dfIMD.iloc [j] ['imu_angvel_z'])
                    mspeed.append (dfIMD.iloc [j] ['max_speed'])

                if len (rgb_dt) == 0: continue                          # Because there were not enough previous samples
              
                # Assert sequence length
                if not len (rgb_dt) == self._cnf.seq_len:
                    raise AssertionError (f"Sequence length not as specified")
                
                ss_cnt = self._cnf.seq_stp                           # Make a sequential step since the sample was added


                # Preprocess rgbs to make training faster
                for j, raw in enumerate (rgb_fp):                       # Iterate through the rgb filepaths of the sequence
                    prc = RGB/Path (raw).name                   # path to the processed rgb
                    rgb_fp [j] = str(prc)                               # overwrite the path to the raw rgb

                    if not prc.is_file (): cv2.imwrite (                # create processed rgb if not created from prior sequences
                        rgb_fp [j], 
                        self.INF_preprocRGB (cv2.imread (str (raw)))
                    )

                # Data frame containing the input sequences and the label
                df_seq = pd.DataFrame (pd.Series (
                    data = [
                        rgb_dt, rgb_fp, imu_dt,
                        imu_lx, imu_ly, imu_lz,
                        imu_ax, imu_ay, imu_az,
                        mspeed,
                        dfIMD.iloc [i] ['exp_waypoint_x'],
                        dfIMD.iloc [i] ['exp_waypoint_y'],
                        dfIMD.iloc [i] ['exp_normspeed'],
                        dfIMD.iloc [i] ['ctrlcmd_bodyrates_x'],
                        dfIMD.iloc [i] ['ctrlcmd_bodyrates_y'],
                        dfIMD.iloc [i] ['ctrlcmd_bodyrates_z'],
                        dfIMD.iloc [i] ['ctrlcmd_angacc_x'],
                        dfIMD.iloc [i] ['ctrlcmd_angacc_y'],
                        dfIMD.iloc [i] ['ctrlcmd_angacc_z'],
                        dfIMD.iloc [i] ['ctrlcmd_collthrust'],
                    ], 
                    index = self.PRC_COLS
                )).transpose ()

                # Add the above data frame
                dfSEQ = pd.concat ([dfSEQ, df_seq], ignore_index=True)
        
        dfSEQ.to_csv (SEQ, index=False)
        eis = float (ei_cnt) / len (dfIMD)

        # LOGGING 
        self.log (f"# samples: {len (dfSEQ)}", log_lvl)
        self.log (f"Expert intervention share: {100 * eis} %", log_lvl)
        self.log_save (SEQ, log_lvl)

        return dfSEQ, eis
        
        



    

    
        

    ##############################################################################################################
    ##############################################################################################################

    def switchInferTopics (self, register : bool) -> None:
        if register:
            self.h = self.model.getZeroInitializedHiddenState (BATCH_SIZE_INFER, TORCH_DEVICE)
            self.rgb = None                                     # lastly fetched rgb image from drone's onboard camera
            self.rgb_t_last = 0                                 # time stamp of previously fetched rgb image
            self.imu_t_last = 0                                 # time stamp of previously fetched imu data
            self.opt = None                                     # ?
            self.opt_t_last = 0                                 # ?
            self.infCnt = 0                                     # number of inferences
            self.aggT01 = 0                                     # aggregated time to process inputs during inference
            self.aggT12 = 0                                     # aggregated time to forward during inference
            self.bridge = cv_bridge.CvBridge ()
            self.sub_RGB = rospy.Subscriber ("/flightmare/rgb", sensor_msgs.msg.Image, self.cb_RGB, queue_size=1)
            self.sub_IMU = rospy.Subscriber ("ground_truth/imu", sensor_msgs.msg.Imu, self.cb_IMU, queue_size=1)
            self.sub_INF = rospy.Subscriber ("brain/trigger_inference", std_msgs.msg.Empty, self.cb_INF)
            self.pub_OUT = rospy.Publisher ("brain/output", geometry_msgs.msg.Point, queue_size=1)
        else:
            self.h = None
            self.rgb = None
            self.rgb_t_last = None
            self.imu_t_last = None
            self.opt = None
            self.opt_t_last = None
            self.infCnt = None
            self.aggT01 = None
            self.aggT12 = None
            self.bridge = None
            try:
                self.sub_RGB.unregister ()
                self.sub_IMU.unregister ()
                self.sub_INF.unregister ()
                self.pub_OUT.unregister ()
            except:
                pass
            self.sub_RGB = None
            self.sub_IMU = None
            self.sub_INF = None
            self.pub_OUT = None

        
    # START TRAINING

    def startTrain (self, num_epochs : int, reset_lrsched : bool) -> None:
        self.asrExpInit ()
        self.switchInferTopics (register=False)



        self.log (f"START TRAIN", 0)
        self.log_load (self._prc.load (pmn=True), 2)
        self.log_load (self._bldRec.load (pmn=True), 2)
        self.log_load (self._trnRec.load (pmn=True), 2)
        self.log_load (self._cpt.load(pmn=True), 2)
        self.loadCheckpoint (reset_lrsched)
        
        self.log (f"Experiment", 1)
        self.log (f"ID: {self.expID}", 2)
        self.log (f"# runs: {self._bldRec.num_records ()}", 2)
        self.log (f"# epochs: {self._trnRec.num_records ()}", 2)





        log (1, '...')
        self.dataloader = torch.utils.data.DataLoader (
            dataset=ForgetfulDataset (
                df=self._prc.get (),
                id=self.expID,
                cnn_cols=self._cnf.cnn_inp,
                cat_cols=self._cnf.opt_inp,
                lbl_cols=self._cnf.lables,
                msg_pfx='|  '
            ),
            batch_size=self._cnf.batch_size,
            shuffle=True,
            drop_last=True
        )
        log (1, '...')
        log (1, 'Data loader')
        log (2, f"Batch size: {self._cnf.batch_size}")
        log (2, f"Shuffle & drop last")
        

        log (1, 'Training')
    
        PMN = False
        with tqdm (range (num_epochs), leave=False, desc='      - Epochs   ', position=0, ncols=NCOLS) as pbar:
            pfx = '|    - '
            for i in pbar:
                epochRec = self.startTrain_trainOneEpoch ()

                print ()                
                PMN = True if (i + 1 == num_epochs) else False          # save only permanently after last epoch

                self._trnRec.add (epochRec)                             # save training records
                self.log_save (self._trnRec.save (pmn=PMN), 2)
                
                self._cpt.set (self._trnRec.num_records (), self.model, self.optim, self.loss)    # save checkpoint
                self.log_save (self._cpt.save (pmn=PMN), 2)
                
                for _ in [True, False]:                                 # save annotated and traced script module
                    self.saveScriptModule (ant=_, pmn=PMN, log_lvl=2)
                
                self.saveLossPlt (pmn=PMN, log_lvl=2)                  # save loss plot

            pbar.write(f"      - Epochs   : 100%")
            


        

        


    


    def startTrain_trainOneEpoch (self) -> EpochRecord:
        self.model.train ()
        self.h = self.model.getZeroInitializedHiddenState (self.dataloader.batch_size, TORCH_DEVICE)

        agg_loss = 0.0      # aggregated loss 
        batch_cnt = 0       # number of batches

        with tqdm (self.dataloader, leave=False, desc='|  Batches  ', position=1, ncols=NCOLS) as pbar:
            for batch in pbar:

                batch_cnt += 1
                self.optim.zero_grad ()

                out, self.h = self.model.forward (
                    x_img=batch ['input'] ['cnn'].to (TORCH_DEVICE), 
                    x_cat=batch ['input'] ['cat'].to (TORCH_DEVICE), 
                    h=self.h.data
                )

                batch_loss = self.loss (out, batch ['label'].to (TORCH_DEVICE))   
                try: batch_loss += self.model.get_regularization_term ()
                except: pass
                
                agg_loss += batch_loss.item ()

                batch_loss.backward ()
                self.optim.step ()

        self.lrSched.step ()


        return EpochRecord (agg_loss / batch_cnt, None, self.lrSched.get_last_lr () [0])


    


    ##############################################################################################################
    ##############################################################################################################

    # START INFERENCE

    def startInfer (self, max_speed : float) -> None:
        self.switchInferTopics (register=True)
        self.maxSpeed = max_speed
        self.model.eval ()


    ##############################################################################################################
    ##############################################################################################################

    # OLD
    
    '''def roscb_timer (self, te: rospy.timer.TimerEvent) -> None:
        if te.last_duration is not None and rospy.Duration(te.last_duration) > self.period:
            rospy.logwarn(f"[{rospy.get_name()}]    ROS timer took {te.last_duration} > {self.period} s.")
        self.triggerInference()'''



    '''def buildData (self, rebuild : bool, exist_ok: bool) -> None:
        self.asrExpInit ()

        if rebuild:
            pass
            
        else:
            self.asrRunInit ()

            if exist_ok:
                try: self.startTrain_buildImd (self.runID, overwrite=False, log=True)
                except FileExistsError as e: print (e)
                try: self.startTrain_buildPrcSeq (self.runID, overwrite=False, log=True)
                except FileExistsError as e: print (e)
            else:
                self.startTrain_buildImd (self.runID, overwrite=False, log=True)'''
    


    '''def buildData_addRun (self, exist_ok: bool) -> None:
        self.asrRunInit ()
        if exist_ok:
            try: self.startTrain_buildImd (self.runID, overwrite=False, log=True)
            except FileExistsError as e: print (e)
            try: self.startTrain_buildPrcSeq (self.runID, overwrite=False, log=True)
            except FileExistsError as e: print (e)
        else:
            self.startTrain_buildImd (self.runID, overwrite=False, log=True)'''

        

    '''def buildIntermediateFiles (
        self, 
        run_ids : List [str],
        force_rebuild : bool,
        log : bool
    ) -> None:

        with tqdm(run_ids, desc='|  Intermediate Data', position=0, ncols=NCOLS) as pbar:
            for x in pbar:
                try: 
                    self.startTrain_buildImd (x, overwrite=False, log=log)
                except FileExistsError as e: 
                    print(e)'''


    
    '''def buildData_ProcessedCSV (self, run_id : str or None, log : bool) -> None:
        
        if run_id is None:
            self.dfPrc = pd.DataFrame ()

            for dir in self.d__exp_dat_prc ().subdirs (): self.dfPrc = pd.concat (
                [
                    self.dfPrc, 
                    self.loadSeqDF (dir/'data.csv')
                ], 
                ignore_index=True
            )

        else: # Add run only
            self.dfPrc = self.loadSeqDF (self.f__exp_dat_prc_prc ())

            pd.concat (
                [
                    self.dfPrc, 
                    self.loadSeqDF (self.d_exp_dat_prc_X_csv (run_id))
                ], 
                ignore_index=True
            )

        self.dfPrc.to_csv (self.f__exp_dat_prc_prc (), index=False)'''

    

    '''def buildProcessedData (self, run_id : str or None = None) -> None:
        if run_id is None:
            self.df_prc = pd.DataFrame();
            intermediate_fpaths = sorted([x for x in (self.intermediate_dpath).iterdir() if x.is_file() and x.suffix == self.INTERMEDIATE_FNAME_EXTENSION])
        else:
            if self.processed_fpath.exists(): self.loadProcessed()
            else: self.df_prc = pd.DataFrame();
            intermediate_fpaths = [self.intermediate_fpath(run_id)]
    
    
        with tqdm(intermediate_fpaths, desc='|  Processed Data', position=0, ncols=NCOLS) as i2p_pbar:
    
            for intermediate_fpath in i2p_pbar:
                run_id = intermediate_fpath.stem
                if not (self.processed_dpath/f"{run_id}").exists(): (self.processed_dpath/f"{run_id}").mkdir()
                df_intermediate = pd.read_csv(intermediate_fpath)
    
                with tqdm(range(len(df_intermediate)), desc='|  Sequential',leave=False, position=1, ncols=NCOLS) as s2s_PBAR:
                    exp_iv_cnt = 0
                    added_seq_cnt = 0
    
                    seq_step_cnt = 1
                    for i in s2s_PBAR:
    
                        if seq_step_cnt > 1:
                            seq_step_cnt -= 1
                            continue
                        
                        seq_rgb_dt = []
                        seq_rgb_fpath = []
                        seq_imu_dt = []
                        seq_imu_linacc_x = []
                        seq_imu_linacc_y = []
                        seq_imu_linacc_z = []
                        seq_imu_angvel_x = []
                        seq_imu_angvel_y = []
                        seq_imu_angvel_z = []
                        seq_max_speed = []
    
                        if df_intermediate.iloc[i]['expert_intervened']:
                            exp_iv_cnt += 1
                            
                            for j in range(i + 1 - self.cnf['data']['sequential']['length'], i + 1):
                                if j < 0: break
                                seq_rgb_dt.append(df_intermediate.iloc[j]['rgb_dt'])
                                seq_rgb_fpath.append(df_intermediate.iloc[j]['rgb_fpath'])
                                seq_imu_dt.append(df_intermediate.iloc[j]['imu_dt'])
                                seq_imu_linacc_x.append(df_intermediate.iloc[j]['imu_linacc_x'])
                                seq_imu_linacc_y.append(df_intermediate.iloc[j]['imu_linacc_y'])
                                seq_imu_linacc_z.append(df_intermediate.iloc[j]['imu_linacc_z'])
                                seq_imu_angvel_x.append(df_intermediate.iloc[j]['imu_angvel_x'])
                                seq_imu_angvel_y.append(df_intermediate.iloc[j]['imu_angvel_y'])
                                seq_imu_angvel_z.append(df_intermediate.iloc[j]['imu_angvel_z'])
                                seq_max_speed.append(df_intermediate.iloc[j]['max_speed'])
    
                            if not len(seq_rgb_dt) == 0:
                                assert self.cnf['data']['sequential']['length'] == len(seq_rgb_dt), "len(sequence) != sequence length"
                                added_seq_cnt += 1
    
                                for k, raw_rgb_fpath in enumerate(seq_rgb_fpath):
                                    
                                    rgb_id = Path(raw_rgb_fpath).stem
                                    processed_rgb_fpath = self.processed_dpath/f"{run_id}"/f"{rgb_id}{self.RGB_EXT}"
                                    seq_rgb_fpath[k] = str(processed_rgb_fpath)
    
                                    if not processed_rgb_fpath.exists():
                                        rgb = cv2.imread(raw_rgb_fpath)
                                        rgb = self.preprocRGB(rgb)
                                        cv2.imwrite(str(processed_rgb_fpath), rgb)
    
      
                                df_sequential = pd.DataFrame(
                                    pd.Series(
                                        data = [
                                            seq_rgb_dt,
                                            seq_rgb_fpath,
                                            seq_imu_dt,
                                            seq_imu_linacc_x,
                                            seq_imu_linacc_y,
                                            seq_imu_linacc_z,
                                            seq_imu_angvel_x,
                                            seq_imu_angvel_y,
                                            seq_imu_angvel_z,
                                            seq_max_speed,
                                            df_intermediate.iloc[i]['exp_waypoint_x'],
                                            df_intermediate.iloc[i]['exp_waypoint_y'],
                                            df_intermediate.iloc[i]['exp_normspeed'],
                                            df_intermediate.iloc[i]['ctrlcmd_bodyrates_x'],
                                            df_intermediate.iloc[i]['ctrlcmd_bodyrates_y'],
                                            df_intermediate.iloc[i]['ctrlcmd_bodyrates_z'],
                                            df_intermediate.iloc[i]['ctrlcmd_angacc_x'],
                                            df_intermediate.iloc[i]['ctrlcmd_angacc_y'],
                                            df_intermediate.iloc[i]['ctrlcmd_angacc_z'],
                                            df_intermediate.iloc[i]['ctrlcmd_collthrust'],
                                        ], 
                                        index = self.PRC_COLS
                                    )
                                ).transpose()
    
                                self.df_prc = pd.concat([self.df_prc, df_sequential], ignore_index=True)
                        
                                seq_step_cnt = self.cnf['data']['sequential']['step']
                    
                    self.rec['expert_intervention_share'].append(1.0 * exp_iv_cnt / len(df_intermediate))
                    s2s_PBAR.write(f"|    [Run {run_id}]  Added #{added_seq_cnt} sequences from # {len(df_intermediate)} samples to processed data")
    
                if intermediate_fpath == intermediate_fpaths[-1]:
                    msg = f"|    -> Wrote #{len(self.df_prc)} sequences to \"{self.relpath(self.processed_fpath)}\""
                    i2p_pbar.write(msg)
            
            
            self.df_prc.to_csv(self.processed_fpath, index=False)
            #self.loadProcessedData()'''

    

    '''def train_from_scratch (self, exp_id:str) -> None:
        self.exp_dpath = self.PKG_DIR/self.EXP_DNAME/exp_id
        self.intermediate_dpath = self.exp_dpath/self.DATA_DNAME/self.INTERMEDIATE_DNAME
        self.processed_dpath = self.exp_dpath/self.DATA_DNAME/self.PROCESSED_DNAME
        self.output_dpath = self.exp_dpath/self.OUTPUT_DNAME
        self.intermediate_dpath.rmtree(); self.intermediate_dpath.mkdir()
        self.processed_dpath.rmtree(); self.processed_dpath.mkdir()
        self.output_dpath.rmtree(); self.output_dpath.mkdir()
        self.raw_dpath = self.exp_dpath/self.DATA_DNAME/self.RAW_DNAME
        for run_id in sorted([x.name for x in self.raw_dpath.iterdir() if x.is_dir()]):
            self.train_ann (
                experiment_dpath=self.exp_dpath,
                latest_run_id=run_id
            )'''


    '''def train_ANN_variant (self, exp_id:str, data_built : bool) -> None:
        self.exp_dpath = self.PKG_DIR/self.EXP_DNAME/exp_id
        self.intermediate_dpath = self.exp_dpath/self.DATA_DNAME/self.INTERMEDIATE_DNAME
        self.processed_dpath = self.exp_dpath/self.DATA_DNAME/self.PROCESSED_DNAME
        self.output_dpath = self.exp_dpath/self.OUTPUT_DNAME
        self.raw_dpath = self.exp_dpath/self.DATA_DNAME/self.RAW_DNAME
        self.latest_run_id = sorted([x.name for x in self.raw_dpath.iterdir() if x.is_dir()])[-1]
        experiment_id = self.exp_dpath.name
        self.processed_fpath = self.processed_dpath/f"{experiment_id}{self.PROCESSED_FNAME_EXTENSION}"
        self.checkpoint_fpath = self.output_dpath/self.CHKPNT_FNAME
    
        run_ids = sorted([x.name for x in self.raw_dpath.iterdir() if x.is_dir()])
        if data_built: run_ids = [run_ids[-1]]
        
        for self.latest_run_id in run_ids:
            if self.latest_run_id not in [x.stem for x in self.intermediate_dpath.iterdir() if x.is_dir()]:
    
                print('\n\n.--- TRAIN FORGETFUL ANN\n|')
                print(f"|  Experiment: {experiment_id}")
                self.initDirectory(self.raw_dpath,          msg_prefix='|    - Raw data:          ')
                self.initDirectory(self.intermediate_dpath, msg_prefix='|    - Intermediate data: ')
                self.initDirectory(self.processed_dpath,    msg_prefix='|    - Processed data:    ')
                self.initDirectory(self.output_dpath,       msg_prefix='|    - Output data:       ')
                self.initCheckpoint(                        msg_prefix='|    - Checkpoint:        ')
                if not self.init_rec(                 msg_prefix='|    - Recordings:        '): return
                    
                print('|')
                if self.buildIntermediateData (runID=self.latest_run_id):
                    print('|')
                    self.buildProcessedData (RunID=self.latest_run_id)
                    self.plotExpertInterventionShare()
                else:
                    print('|')
                    print('|  Processed Data:')
                    self.loadProcessedData()
                    print(f'|    - Loaded #{len(self.df_processed)} sequences')
                print('|')
    
    
        self.dataset = ForgetfulDataset (
            df=self.df_processed,
            id=experiment_id,
            cnn_cols=self.cnf['data']['input']['cnn'],
            cat_cols=self.cnf['data']['input']['cat'],
            lbl_cols=self.cnf['data']['label'],
            msg_pfx='|  '
        )
        print('|')
    
        bs = self.cnf['learn']['batch_size']
        sffl = True
        dl = True
        self.dataloader = torch.utils.data.DataLoader (
            dataset=self.dataset,
            batch_size=bs,
            shuffle=sffl,
            drop_last=dl
        )
        print(f'|  Data Loader')
        print(f'|    - batch size: {bs}')
        print(f'|    - shuffle: {sffl}')
        print(f'|    - drop last: {dl}')
        print(f'|')
    
    
    
        log_savings = True
        with tqdm(range(self.cnf['learn']['num_epochs']), desc='|  Epochs   ', position=0, ncols=NCOLS) as train_PBAR:
            pfx = '|    - '
            for self.epoch_i in train_PBAR:
    
                try: 
                    self.recordings['learn_rate'][-1][self.epoch_i]
                    self.optimizer.step()
                    self.lr_scheduler.step()
                    print(self.lr_scheduler.get_last_lr())
                    continue
                except:
                    pass
                
                self.train_one_epoch ()
                
                self.export_dict_as_json (
                    dictionary=self.user_input, 
                    fname=self.UI_FNAME,
                    log = log_savings,
                    pbar=train_PBAR,
                    msg_prefix=pfx
                )
                self.export_dict_as_json (
                    dictionary=self.cnf, 
                    fname=self.CONFIG_FNAME,
                    log = log_savings,
                    pbar=train_PBAR,
                    msg_prefix=pfx
                )
                self.export_dict_as_json (
                    dictionary=self.recordings, 
                    fname=self.REC_FNAME,
                    log = log_savings,
                    pbar=train_PBAR,
                    msg_prefix=pfx
                )
                self.save_recordings_plots (
                    log = log_savings,
                    pbar=train_PBAR,
                    msg_prefix=pfx
                )
                self.save_checkpoint (
                    log = log_savings,
                    pbar=train_PBAR, 
                    msg_prefix=pfx
                )
                for ant in [True, False]: self.saveScriptModule(ant=ant, log_lvl=2)
                log_savings = False
    
                train_PBAR.write(f"|    [Epoch {self.epoch_i:04d}]  Loss: {self.recordings['loss']['train'][self.num_prev_runs][-1]:.10f} (train) | {'None'} (valid)  //  LR:  {self.recordings['learn_rate'][self.num_prev_runs][-1]:.10f}")      
    
                if self.epoch_i + 1 == self.cnf['learn']['num_epochs']: train_PBAR.write("|", end='\n')
        print("|_________________________________________________")'''


    '''def save_recordings_plots (self, log:bool=True, pbar:tqdm=None, msg_prefix:str='') -> None:        
        fpath1 = self._exp_out_/self.REC_PLOT_LIN_FNAME 
        fpath2 = self._exp_out_/self.REC_PLOT_LOG_FNAME  
        
        try:
            plt.rcParams['text.usetex'] = True
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                fig, ax1 = plt.subplots(figsize=(10, 7))
            color = 'tab:orange'
            ax1.set_xlabel('Epochs')
            ax1.set_ylabel(self.cnf['learn']['loss']['id'], color=color)

            epoch_cnt = 1
            for loss in self.rec['loss']['train']:
                label = 'Training' if (epoch_cnt == 1) else None
                x_axis = list(range(epoch_cnt, epoch_cnt + len(loss)))
                epoch_cnt += len(loss)
                plt.plot(x_axis, loss, color=color, linestyle=':', label=label)
            
            epoch_cnt = 1
            for loss in self.rec['loss']['valid']:
                label = 'Validation' if (epoch_cnt == 1) else None
                x_axis = list(range(epoch_cnt, epoch_cnt + len(loss)))
                epoch_cnt += len(loss)
                plt.plot(x_axis, loss, color=color, linestyle='-', label=label)

            ax1.tick_params(axis='y', labelcolor=color)
            #ax1.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.3e'))

            plot_lr = False
            if plot_lr:                
                ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
                color = 'tab:blue'
                ax2.set_ylabel('Learning Rate', color=color)  # we already handled the x-label with ax1
                
                epoch_cnt = 1
                for learn_rate in self.rec['learn_rate']:
                    label = None if (epoch_cnt != 1) else\
                        f"{self.cnf['learn']['lr_scheduler']['id']}, $\gamma={self.cnf['learn']['lr_scheduler']['gamma']}$"
                    x_axis = list(range(epoch_cnt, epoch_cnt + len(learn_rate)))
                    epoch_cnt += len(learn_rate)
                    ax2.plot(x_axis, learn_rate, color=color, label=label)
                
                ax2.tick_params(axis='y', labelcolor=color)
                ax2.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.1e'))

            ax1.legend(loc='lower left')
            if plot_lr: ax2.legend(loc='upper right')
            fig.tight_layout()  # otherwise the right y-label is slightly clipped
             
            plt.savefig(fpath1)

            ax1.set_yscale('log')
            if plot_lr: ax2.set_yscale('log')
            
            plt.savefig(fpath2)
            #plt.close()

            msg1 = f"{msg_prefix}Save plot as \"{self._rp(fpath1)}\""
            msg2 = f"{msg_prefix}Save plot as \"{self._rp(fpath2)}\""

        except Exception as e:
            msg1 = f"{msg_prefix}Failed to save plot as \"{self._rp(fpath1)}\", error: {e}"
            msg2 = f"{msg_prefix}Failed to save plot as \"{self._rp(fpath2)}\", error: {e}"
        
        if log:
            print(msg1) if (pbar is None) else pbar.write(msg1)
            print(msg2) if (pbar is None) else pbar.write(msg2)'''



        

    ##############################################################################################################
    ##############################################################################################################



    
    
    
    




    
        

    def export_dict_as_json (self, dictionary:Dict, fname:str, log:bool=True, pbar:tqdm=None, msg_prefix:str='') -> None:
        fpath = self._exp_out_/fname
        
        try:
            with open(fpath, 'w') as file:
                file.write(json.dumps(dictionary, sort_keys=True, indent=4))
            msg = f"{msg_prefix}Export dict to \"{self._rp(fpath)}\""

        except Exception as e:
            msg = f"{msg_prefix}Failed to export dict to \"{self._rp(fpath)}\", error: {e}"
        
        if log: print(msg) if (pbar is None) else pbar.write(msg)




    

    ##############################################################################################################
    ##############################################################################################################

    # DURING INFERENCE

    def INF (self) -> None:
        self.infCnt += 1
        
        t_0 = rospy.Time.now ()

        if not self.INF_fetchRGB (): return
        if not self.INF_fetchIMU (): return
        x_rgb = self.INF_tensorRGB ()
        x_opt = self.INF_tensorOPT ()

        t_1 = rospy.Time.now ()

        x_out, self.h = self.model.forward (x_img=x_rgb, x_cat=x_opt, h=self.h.data)
        out = x_out [0].detach ().cpu ().numpy ()

        t_2 = rospy.Time.now()

        T_01 = (t_1 - t_0).to_sec()
        T_12 = (t_2 - t_1).to_sec()
        self.aggT01 += T_01
        self.aggT12 += T_12

        rospy.loginfo_throttle (5.0, 
            f"[{rospy.get_name()}]    Inference time [ms]:  - input proc. {1000*T_01}  - forward {1000*T_12}"
        )

        self.INF_pubOUT (out)
    

    def INF_fetchRGB (self) -> bool:
        try: 
            self.rgb = self.bridge.imgmsg_to_cv2 (self.rgb_mgs, "bgr8")
            self.rgb_dt = self.rgb_mgs.header.stamp.to_sec () - self.rgb_t_last
            self.rgb_t_last = self.rgb_mgs.header.stamp.to_sec ()
            return True
        except cv_bridge.CvBridgeError as e:
            rospy.logerr (e)
            self.rgb = None
            return False
        

    def INF_fetchIMU (self) -> bool:
        try: 
            self.imu = np.array (
                (
                    self.imu_msg.linear_acceleration.x,
                    self.imu_msg.linear_acceleration.y,
                    self.imu_msg.linear_acceleration.z,
                    self.imu_msg.angular_velocity.x,
                    self.imu_msg.angular_velocity.y,
                    self.imu_msg.angular_velocity.z
                ),
                dtype=np.float32
            )
            self.imu_dt = self.imu_msg.header.stamp.to_sec () - self.imu_t_last
            self.imu_t_last = self.imu_msg.header.stamp.to_sec ()
            return True
        except Exception as e:
            rospy.logerr (e)
            self.imu = None
            return False
    
    def INF_preprocRGB (self, rgb : np.ndarray) -> np.ndarray:
        rgb = cv2.resize(
            rgb, 
            (
                self._cnf.prc_rgb_W,
                self._cnf.prc_rgb_H
            )
        )
        return cv2.cvtColor (rgb, cv2.COLOR_BGR2RGB)

    def INF_tensorRGB (self) -> torch.Tensor:
        C = self._cnf.prc_rgb_N
        H = self._cnf.prc_rgb_H
        W = self._cnf.prc_rgb_W
        
        rgb = self.INF_preprocRGB(self.rgb)
        rgb = np.array(rgb / 255.0, dtype=np.float32)
        rgb = np.transpose(rgb, (2, 0, 1)) # channel size to index 0
        rgb = torch.tensor(rgb, dtype=torch.float32)
        rgb = rgb.view(1, 1, C, H, W).to(TORCH_DEVICE)
        
        return rgb

    def INF_tensorOPT (self) -> torch.Tensor:
        N = len (self._cnf.opt_inp)
        
        opt = np.concatenate (
            (
                np.array ([self.rgb_dt, self.imu_dt]), 
                self.imu,
                np.array ([self.maxSpeed])
            )
        ) [self.optMask]
        opt = torch.tensor (opt, dtype=torch.float32)
        opt = opt.view (1, 1, N).to (TORCH_DEVICE)

        return opt


    def INF_pubOUT (self, out : np.ndarray) -> None:
        self.pub_OUT.publish (
            geometry_msgs.msg.Point (
                x=out [0], y=out [1], z=out [2]
        ))
        

    ##############################################################################################################
    ##############################################################################################################

    # REBUILD DATA

    def rebuildRuns (self) -> None:
        raw_runs = self._exp_dat_raw_ ().subdirs (sort=True)

        with tqdm (raw_runs, desc='|  Runs', leave=False, position=0, ncols=NCOLS) as pbar:

            for _raw_run_ in pbar:
                self.buildRun (_raw_run_.name)


    def initExp_resetTraining (self, id) -> None:
        self.log ('INIT EXPERIMENT', 0)
        
        self.expID = id
        self.log (f"ID: {self.expID}", 1)

        if not self._exp_cnf_CNF ().is_file ():
            raise ValueError ("No config provided")

        self.log (f"Load existing - reset training", 1)
        
        LOGLVL = 2
        PMN = True
        
        # ASSERT DIRS/FILES  EXIST
        for isdir in [True, False]:
            for path in self._exExp_ (isdir=isdir):
                self.asrPath (path, isdir=isdir, exists=True)

        # LOAD FILES
        self._cnf = ConfigFile (                    # configuration File
            self._exp_cnf_CNF (), 
            None,
            new=False
        )
        self.log_load (
            self._cnf.load (pmn=PMN), LOGLVL)
        
        self._prc = DataFrameFile (                 # processed data file
            self._exp_dat_prc_PRC (), 
            self._exp_out_tmp_PRC (), 
            self.PRC_SEQ_COLS
        )
        self.log_load (
            self._prc.load (pmn=PMN), LOGLVL)

        self._bldRec = RecordsFile (                # data building recordings
            self._exp_out_BRC (), 
            self._exp_out_tmp_BRC (),
            RunRecord.id
        )
        self.log_load (
            self._bldRec.load (pmn=PMN), LOGLVL)

        self._trnRec = RecordsFile (                # training recordings
            self._exp_out_TRC (), 
            self._exp_out_tmp_TRC (),
            EpochRecord.id
        )
        self.log_create (
            self._trnRec.save (pmn=PMN), LOGLVL)

        # INIT MODEL ETC
        self.initModelEtc ()

        # SAVE CHECKPOINT
        self._cpt = CheckpointFile (
            self._exp_out_CPT (),
            self._exp_out_tmp_CPT ()
        )
        self._cpt.set (
            0,
            self.model,
            self.optim,
            self.loss
        )
        self.log_create (
            self._cpt.save (pmn=PMN), LOGLVL)








    
def debug ():
    rospy.set_param ('SIM_UNITY_DRONE_CAMERA_WIDTH', 720)
    rospy.set_param ('SIM_UNITY_DRONE_CAMERA_HEIGHT', 480)
    rospy.set_param ('MAIN_FREQ', 50.0)

    fb = ForgetfulBrain ()

    # Init Experiment
    req = fdsrv.StringRequest ()
    req.data = 'DEBUG_NEW_EXP'
    res = fb.cb_initExp (req)

    # Build Run
    req = fdsrv.StringRequest ()
    req.data = '0000___2_0_1_1_0_1_04.00_00.00___000'
    res = fb.cb_buildRun (req)

    # Start Training
    req = fdsrv.IntRequest ()
    req.data = 10
    res = fb.cb_startTrain (req)

    # Start Inference
    req = fdsrv.FloatRequest ()
    req.data = 3.141
    res = fb.cb_startInfer (req)

    

    
    #dagger.train_ANN_variant(exp_id='UTC_2022_7_1_7_27_56___SEQLEN_x', data_built=False)
    #dagger.train_ann(
    #    experiment_dpath=Path('/home/fm/catkin_ws/src/forgetful_drone/experiments/UTC_2022_6_13_20_21_51'),
    #    latest_run_id='0002___0_0_0_1_0_1_04.00_00.70___002'
    #)




if __name__ == '__main__':
    if DEBUG: debug ()
    else:
        rospy.init_node ('forgetful_brain')
        fb = ForgetfulBrain ()
        rospy.spin()