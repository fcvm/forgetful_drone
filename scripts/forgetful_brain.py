#!/usr/bin/env python3
import warnings
import ast

from distutils.command.config import config
from sre_constants import SUCCESS
import numpy as np
import rospy
import rospkg; rospack = rospkg.RosPack()

import std_msgs.msg
import std_srvs.srv

import sensor_msgs.msg
import geometry_msgs.msg
import cv_bridge

import pathlib
import json
import torch
import cv2
from tqdm import tqdm

#from forgetful_drones.srv import StringTrigger, StringTriggerRequest, StringTriggerResponse, Inference, InferenceRequest, InferenceResponse

from forgetful_drones.srv import TrainBrain, TrainBrainRequest, TrainBrainResponse



from typing import List, Dict, Any
import pandas as pd
import copy


import forgetful_ann
import forgetful_dataset

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick

import user_input




def name_prefix (path: pathlib.Path) -> str:
    return path.name.split(sep='_')[0]
setattr(pathlib.Path, "name_prefix", name_prefix)

def name_has_prefix (path: pathlib.Path, prefix: str) -> bool:
    return path.name_prefix() == prefix
setattr(pathlib.Path, "name_has_prefix", name_has_prefix)

def rmtree (path: pathlib.Path) -> None:
    for subpath in path.iterdir():
        if subpath.is_file(): subpath.unlink()
        else: rmtree(subpath)
    path.rmdir()
setattr(pathlib.Path, "rmtree", rmtree)

def dict2json (dictionary: Dict) -> str:
    return json.dumps(dictionary, sort_keys=True, indent=4)

TORCH_DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
INFER_EVERY_INCOMING_FRAME = False
NCOLS = 0



class DAGGER:

    def print_user_input (self) -> None:
        print(dict2json(self.user_input))
    def print_config (self) -> None:
        print(dict2json(self.config))

        
    def __init__(self, user_input: Dict):
        
        self.user_input = user_input
        self.config = self.initConfig(user_input)
        
        self.CHECKPOINT_FNAME = 'checkpoint.pt'
        self.PACKAGE_DPATH = pathlib.Path(rospack.get_path('forgetful_drones'))
        

        self.OUTPUT_DNAME = 'output'
        self.DATA_DNAME = 'data'
        self.RAW_DNAME = 'raw'
        self.INTERMEDIATE_DNAME = 'intermediate'
        self.PROCESSED_DNAME = 'processed'
        self.EXPERIMENTS_DNAME = 'experiments'

        self.RGB_DNAME = 'rgb'
        self.RGB_FNAME_EXTENSION = '.jpg'
        self.DATA_FNAME = 'data.txt'
        self.INTERMEDIATE_FNAME_EXTENSION = '.csv'
        self.PROCESSED_FNAME_EXTENSION = '.csv'

        self.DATA_INPUT_CAT = [
            'rgb_dt',
            'imu_dt',
            'imu_linacc_x',
            'imu_linacc_y',
            'imu_linacc_z',
            'imu_angvel_x',
            'imu_angvel_y',
            'imu_angvel_z',
            'max_speed'
        ]

        self.RGB_COLS = ['rgb_fpath']
        self.RAW_DATA_COLS = [
            'expert_intervened',
            'rgb_dt',
            'imu_dt',
            'imu_linacc_x',
            'imu_linacc_y',
            'imu_linacc_z',
            'imu_angvel_x',
            'imu_angvel_y',
            'imu_angvel_z',
            'max_speed',
            'exp_waypoint_x',
            'exp_waypoint_y',
            'exp_normspeed',
            'ctrlcmd_bodyrates_x',
            'ctrlcmd_bodyrates_y',
            'ctrlcmd_bodyrates_z',
            'ctrlcmd_angacc_x',
            'ctrlcmd_angacc_y',
            'ctrlcmd_angacc_z',
            'ctrlcmd_collthrust'
        ]
        self.INTERMEDIATE_DATA_COLS = [
            *self.RAW_DATA_COLS[:2],
            *self.RGB_COLS,
            *self.RAW_DATA_COLS[2:],
        ]
        self.PROCESSED_DATA_COLS = self.INTERMEDIATE_DATA_COLS[1:]
        self.PROCESSED_DATA_SEQ_COLS = self.PROCESSED_DATA_COLS[:10]
        self.USER_INPUT_FNAME = 'user_input.json'
        self.CONFIG_FNAME = 'config.json'
        self.RECORDINGS_FNAME = 'recordings.json'
        self.RECORDINGS_PLOT_LIN_FNAME = 'recordings_lin_y.pdf'
        self.RECORDINGS_PLOT_LOG_FNAME = 'recordings_log_y.pdf'
        self.ANNOTATED_FNAME = 'model_scripted_with_annotation.pt'
        self.TRACED_FNAME = 'model_scripted_with_tracing.pt'
        self.EXPERT_INTERVENTION_SHARE_PLOT_FNAME = 'expert_intervention_share.pdf'


        self.model = self.initModel()
        self.optimizer = self.initOptimizer()
        self.lr_scheduler = self.initLRScheduler()
        self.loss = self.initLoss()

        self.h = None
        self.rgb = None
        self.rgb_t_last = 0
        self.imu_t_last = 0
        self.cat = None
        self.cat_t_last = 0
        self.cat_mask = self.initCatMask()
        self.bridge = cv_bridge.CvBridge()
        self.output = None


        self.rossub_flightmare_rgb = rospy.Subscriber("/flightmare/rgb", sensor_msgs.msg.Image, self.roscb_flightmare_rgb, queue_size=1)
        self.rossub_ground_truth_imu = rospy.Subscriber("ground_truth/imu", sensor_msgs.msg.Imu, self.roscb_ground_truth_imu, queue_size=1)
        self.rospub_brain_output = rospy.Publisher("brain/output", geometry_msgs.msg.Point, queue_size=1)
        
        self.rossvs_train_ann = rospy.Service("brain/train_ann", std_srvs.srv.Empty, self.roscb_train_ann)
        
        self.rossub_load_checkpoint = rospy.Subscriber("brain/load_checkpoint", std_msgs.msg.String, self.roscb_load_checkpoint)
        #self.rossub_save_checkpoint = rospy.Service("brain/save_checkpoint", StringTrigger, self.roscb_save_checkpoint)

        self.rossub_enable_inference = rospy.Subscriber("brain/enable_inference", std_msgs.msg.Bool, self.roscb_enable_inference, queue_size=1)
        self.rossub_trigger_inference = rospy.Subscriber("brain/trigger_inference", std_msgs.msg.Empty, self.roscb_trigger_inference)


    def roscb_trigger_inference (self, msg:std_msgs.msg.Empty) -> None:
        self.roscb_infer_once()

    



    def initCatMask (self) -> List[int]:
        mask = []
        for cat_i in self.config['data']['input']['cat']:
            for j, CAT_J in enumerate(self.DATA_INPUT_CAT):
                if cat_i == CAT_J: 
                    mask.append(j)
                    break
        return mask

        



    def initConfig (self, user_input: Dict[str, Dict[str, Any]]) -> Dict:        
        c = copy.deepcopy(user_input)
        
        # data/raw/rgb/...
        try: c['data']['raw']
        except KeyError: c['data']['raw'] = {}
        try: c['data']['raw']['rgb']
        except KeyError: c['data']['raw']['rgb'] = {}
        
        c['data']['raw']['rgb']['num_channels'] = 3
        
        try: c['data']['raw']['rgb']['width'] = rospy.get_param('SIM_UNITY_DRONE_CAMERA_WIDTH')
        except Exception as e: print(e); successful = False;# c['data']['raw']['rgb']['width'] = 720
        
        try: c['data']['raw']['rgb']['height'] = rospy.get_param('SIM_UNITY_DRONE_CAMERA_HEIGHT')
        except Exception as e: print(e); successful = False;# c['data']['raw']['rgb']['height'] = 480
        
        try: c['data']['raw']['nominal_rate'] = rospy.get_param('DRONE_MAIN_LOOP_FREQUENCY')
        except Exception as e: print(e); successful = False;# c['data']['raw']['rgb']['rate'] = 50

        # data/processed/rgb/...
        rf = c['data']['processed']['rgb']['resize_factor']
        c['data']['processed']['rgb']['num_channels'] = c['data']['raw']['rgb']['num_channels']
        c['data']['processed']['rgb']['width'] = int(rf * c['data']['raw']['rgb']['width'])
        c['data']['processed']['rgb']['height'] = int(rf * c['data']['raw']['rgb']['height'])


        # ann/cat/input_size
        data_input_cat = c['data']['input']['cat']
        assert isinstance(data_input_cat, list), 'Wrong type'
        c['ann']['cat']['input_size'] = None if (len(data_input_cat) == 0) else len(data_input_cat)
        
        # ann/head/output_size
        data_label = c['data']['label']
        assert isinstance(data_label, list), 'Wrong type'
        len_data_label = len(data_label)
        assert len_data_label > 0, 'User input: all label columns disabled'
        c['ann']['head']['output_size'] = len_data_label

        # ann/gru/...
        data_sequential_length = c['data']['sequential']['length']
        assert isinstance(data_sequential_length, int), 'Wrong type'
        assert data_sequential_length > 0, 'User input: sequential length < 1'
        if data_sequential_length == 1:
            c['ann']['gru']['num_layers'] = None
            c['ann']['gru']['hidden_size'] = None
            c['ann']['gru']['dropout'] = None

        # ann/fc/...
        ann_fc_numlayers = c['ann']['fc']['num_layers']
        if ann_fc_numlayers == None:
            c['ann']['fc']['width'] = None
            c['ann']['fc']['activation_function_id'] = None
            c['ann']['fc']['dropout'] = None
        else:
            assert isinstance(ann_fc_numlayers, int), 'Wrong type'
            assert ann_fc_numlayers > 0, 'User input: number of layers of fully connected < 1'

        return c

    def initModel (self) -> forgetful_ann.ForgetfulANN:
        config_ann = self.config['ann']
        return forgetful_ann.ForgetfulANN(config_ann).to(TORCH_DEVICE)
    def initOptimizer (self) -> torch.optim.Optimizer:
        lr_init = self.config['learn']['lr_scheduler']['init']
        optimizer_id = self.config['learn']['optimizer']['id']
        try: return {
                'Adam': torch.optim.Adam(self.model.parameters(), lr=lr_init),
                'SGD': torch.optim.SGD(self.model.parameters(), lr=lr_init),
            }[optimizer_id]
        except: raise ValueError(f"Unknown optimizer id: {optimizer_id}.")
    def initLRScheduler (self) -> torch.optim.lr_scheduler._LRScheduler:
        optimizer = self.optimizer
        lr_gamma = self.config['learn']['lr_scheduler']['gamma']
        lr_scheduler_id = self.config['learn']['lr_scheduler']['id']
        try: return {
                'ExponentialLR': torch.optim.lr_scheduler.ExponentialLR(optimizer, gamma=lr_gamma)
            }[lr_scheduler_id]
        except:
            raise ValueError(f"Unknown learning rate scheduler id: {lr_scheduler_id}.")
    def initLoss (self) -> torch.nn.Module:
        loss_id = self.config['learn']['loss']['id']
        try: return {
                'SmoothL1Loss': torch.nn.SmoothL1Loss(),
                'MSELoss': torch.nn.MSELoss()
            }[loss_id]
        except: raise ValueError(f"Unknown loss id: {loss_id}.")
    
    def initRecordings (self, msg_prefix:str='') -> bool:
        success = True
        fpath = self.output_dpath/self.RECORDINGS_FNAME 
        if fpath.exists():
            with open(fpath, 'r') as file:
                self.recordings = json.load(file)
            
            if self.latest_run_id in self.recordings['run_id']:
                if self.config['learn']['num_epochs'] <= len(self.recordings['loss']['train'][-1]):
                    msg = f"{msg_prefix}already trained on specified run \"{self.latest_run_id}\"! Do nothing!"
                    success = False
                else:
                    self.num_prev_runs = 0
                    msg = f"{msg_prefix}found #{self.num_prev_runs} previous runs"
            else:
                self.num_prev_runs = len(self.recordings['loss']['train'])
                msg = f"{msg_prefix}found #{self.num_prev_runs} previous runs"
                self.recordings['loss']['train'].append([])
                self.recordings['loss']['valid'].append([])
                self.recordings['learn_rate'].append([])
                self.recordings['run_id'].append(self.latest_run_id)
        
        else:
            self.recordings = {
                'run_id': [self.latest_run_id],
                'expert_intervention_share': [],
                'loss': {
                    'train': [[]],
                    'valid': [[]],
                },
                'learn_rate': [[]],
                
            }
            self.num_prev_runs = 0
            msg = f"{msg_prefix}found no previous runs"
        
        print(msg)
        return success


    

    def rel (self, path:pathlib.Path) -> pathlib.Path:
        return path.relative_to(self.PACKAGE_DPATH)


    def roscb_load_checkpoint (self, msg:std_msgs.msg.String) -> None:
        exp_id = msg.data
        checkpoint_fpath = self.PACKAGE_DPATH/self.EXPERIMENTS_DNAME/exp_id/self.OUTPUT_DNAME/self.CHECKPOINT_FNAME
        self.load_checkpoint(checkpoint_fpath)



    def roscb_enable_inference (self, msg: std_msgs.msg.Bool) -> None:
        self.enable_inference (msg.data)

    def enable_inference (self, enabled:bool) -> None:
        if enabled:
            print('ENABLE INFERENCE')
            self.model.eval()
            self.h = self.model.getZeroInitializedHiddenState(1, TORCH_DEVICE)
            self.max_speed = rospy.get_param('brain_MAX_SPEED')
        else:
            print('DISABLE INFERENCE')
            self.model.train()
        
    
    

    def createLink2Directory(self, lpath:pathlib.Path, dpath:pathlib.Path, msg_prefix:str='') -> None:
        if not lpath.exists():
            lpath.symlink_to(target=dpath, target_is_directory=True)
            msg_prefix += 'created'
        else:
            assert lpath.resolve() == dpath, f"Link \"{lpath}\" resolves to \"{lpath.resolve()}\" and not to \"{dpath}\" as specified"
            msg_prefix += 'found'

        print(f"{msg_prefix} link \"{self.rel(lpath)}\" to \"{self.rel(dpath)}\"")

    def initDirectory(self, dpath:pathlib.Path, msg_prefix:str='') -> None:
        if not dpath.exists(): 
            dpath.mkdir(); 
            msg_prefix += 'created'
        else:
            assert dpath.is_dir(), f"\"{dpath}\" is not a directory"
            msg_prefix += 'found'

        print(f"{msg_prefix} directory \"{self.rel(dpath)}\"");







    def buildIntermediateData (self, runID:str or None = None) -> bool:
        run_dpaths = sorted([x for x in self.raw_dpath.iterdir() if x.is_dir()]) if (runID is None) else [self.raw_dpath/runID]

            
        with tqdm(run_dpaths, desc='|  Intermediate Data', position=0, ncols=NCOLS) as r2i_PBAR:
            
            for run_dpath in r2i_PBAR:
                run_id = run_dpath.name
                fpath = self.intermediate_fpath(run_id)
                
                if runID is not None:
                    if fpath.exists():
                        r2i_PBAR.write(f"|    [Run {run_id}]  \"{self.rel(fpath)}\" already exists and is not overwritten!")
                        return False

        
                rgb_dpath = run_dpath/self.RGB_DNAME
                rgb_fpaths = sorted([x for x in rgb_dpath.iterdir() if x.is_file() and x.suffix == self.RGB_FNAME_EXTENSION])
                data_fpath = run_dpath/self.DATA_FNAME


                df_rgb = pd.DataFrame(rgb_fpaths, columns=self.RGB_COLS)
                df_data = pd.read_csv(data_fpath);
                df_data['max_speed'] = df_data['max_speed'].astype(float)
                assert list(df_data.columns) == self.RAW_DATA_COLS, 'Columns of raw data mismatch'
                assert len(df_rgb) == len(df_data), '#frames and #datasamples mismatch'
                

                df_intermediate = pd.concat([df_rgb, df_data], axis=1)
                df_intermediate = df_intermediate[self.INTERMEDIATE_DATA_COLS]

                
                df_intermediate.to_csv(fpath, index=False)
                r2i_PBAR.write(f"|    [Run {run_id}]  Wrote #{len(df_intermediate)} samples to \"{self.rel(fpath)}\"")
        
        return True


    def intermediate_fpath (self, run_id: str) -> pathlib.Path:
        return self.intermediate_dpath/f"{run_id}{self.INTERMEDIATE_FNAME_EXTENSION}"

    
    def loadProcessedData (self) -> None:
        self.df_processed = pd.read_csv(self.processed_fpath)
        
        for i in range(len(self.df_processed)):
            for col in self.PROCESSED_DATA_SEQ_COLS:
                try: 
                    self.df_processed.at[i, col] = json.loads(self.df_processed.at[i, col])
                except json.decoder.JSONDecodeError:
                    fpaths = ast.literal_eval(self.df_processed.at[i, col])
                    #fpaths = re.findall("PosixPath\('(.*?)'\)", self.df_processed.at[i, col])
                    #for j in range(len(fpaths)):
                    #    fpaths[j] = pathlib.Path(fpaths[j])
                    self.df_processed.at[i, col] = fpaths

        

    def preprocessRGB (self, rgb: np.ndarray) -> np.ndarray:
        rgb = cv2.resize(rgb, (
            self.config['data']['processed']['rgb']['width'], 
            self.config['data']['processed']['rgb']['height']
        ))
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        return rgb

        

    def buildProcessedData (self, RunID:str or None = None) -> None:
        if RunID is None:
            self.df_processed = pd.DataFrame();
            intermediate_fpaths = sorted([x for x in (self.intermediate_dpath).iterdir() if x.is_file() and x.suffix == self.INTERMEDIATE_FNAME_EXTENSION])
        else:
            if self.processed_fpath.exists(): self.loadProcessedData()
            else: self.df_processed = pd.DataFrame();
            intermediate_fpaths = [self.intermediate_fpath(RunID)]


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
                            
                            for j in range(i + 1 - self.config['data']['sequential']['length'], i + 1):
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
                                assert self.config['data']['sequential']['length'] == len(seq_rgb_dt), "len(sequence) != sequence length"
                                added_seq_cnt += 1

                                for k, raw_rgb_fpath in enumerate(seq_rgb_fpath):
                                    
                                    rgb_id = pathlib.Path(raw_rgb_fpath).stem
                                    processed_rgb_fpath = self.processed_dpath/f"{run_id}"/f"{rgb_id}{self.RGB_FNAME_EXTENSION}"
                                    seq_rgb_fpath[k] = str(processed_rgb_fpath)

                                    if not processed_rgb_fpath.exists():
                                        rgb = cv2.imread(raw_rgb_fpath)
                                        rgb = self.preprocessRGB(rgb)
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
                                        index = self.PROCESSED_DATA_COLS
                                    )
                                ).transpose()

                                self.df_processed = pd.concat([self.df_processed, df_sequential], ignore_index=True)
                        
                                seq_step_cnt = self.config['data']['sequential']['step']
                    
                    self.recordings['expert_intervention_share'].append(1.0 * exp_iv_cnt / len(df_intermediate))
                    s2s_PBAR.write(f"|    [Run {run_id}]  Added #{added_seq_cnt} sequences from # {len(df_intermediate)} samples to processed data")

                if intermediate_fpath == intermediate_fpaths[-1]:
                    msg = f"|    -> Wrote #{len(self.df_processed)} sequences to \"{self.rel(self.processed_fpath)}\""
                    i2p_pbar.write(msg)
            
            
            self.df_processed.to_csv(self.processed_fpath, index=False)
            #self.loadProcessedData()
            

    


    def train_one_epoch (self) -> None:
        self.model.train()
        loss_sum = 0.0
        batch_cnt = 0

        self.h = self.model.getZeroInitializedHiddenState(self.dataloader.batch_size, TORCH_DEVICE)
        with tqdm(self.dataloader, leave=False, desc='|  Batches  ', position=1, ncols=NCOLS) as batch_PBAR:

            for batch in batch_PBAR:
                batch_cnt += 1
                self.h = self.h.data
                self.optimizer.zero_grad()

                output, self.h = self.model.forward (
                    x_img=batch['input']['cnn'].to(TORCH_DEVICE), 
                    x_cat=batch['input']['cat'].to(TORCH_DEVICE), 
                    h=self.h
                )

                #batch_loss\
                #    = loss(batch_outputs[:, :2], batch_waypoint)\
                #    + loss(batch_outputs[:, 2:], batch_speed) * config.LOSS_SPEED2WAYPOINT_RATIO
                batch_loss = self.loss(output, batch['label'].to(TORCH_DEVICE))
                try:
                    batch_loss += self.model.get_regularization_term()
                except:
                    pass
                loss_sum += batch_loss.item()

                batch_loss.backward()
                self.optimizer.step()

        self.lr_scheduler.step()
        
        self.recordings['loss']['train'][self.num_prev_runs].append(loss_sum / batch_cnt)
        self.recordings['loss']['valid'][self.num_prev_runs].append(None)
        self.recordings['learn_rate'][self.num_prev_runs].append(self.lr_scheduler.get_last_lr()[0])

    def initCheckpoint (self, msg_prefix:str='') -> None:
        if not self.checkpoint_fpath.exists():
            print(f"{msg_prefix}found none, consider this the very first training")
        else:
            self.load_checkpoint(self.checkpoint_fpath, log=False)
            print(f"{msg_prefix}found file \"{self.rel(self.checkpoint_fpath)}\"")


    def plotExpertInterventionShare (self) -> None:        
        fpath = self.output_dpath/self.EXPERT_INTERVENTION_SHARE_PLOT_FNAME 
        
        try:
            plt.rcParams['text.usetex'] = True
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                fig, ax1 = plt.subplots(figsize=(10, 7))
            color = 'tab:green'
            ax1.set_xlabel('Runs')
            ax1.set_ylabel('Expert Interventions [Percentage]', color=color)
            plt.plot(self.recordings['expert_intervention_share'], color=color)
            ax1.tick_params(axis='y', labelcolor=color)
            fig.tight_layout()  # otherwise the right y-label is slightly clipped
            plt.savefig(fpath)
            #plt.close()

        except Exception as e:
            print(f"Failed to save plot as \"{self.rel(fpath)}\", error: {e}")


    def train_from_scratch (self, exp_id:str) -> None:
        self.experiment_dpath = self.PACKAGE_DPATH/self.EXPERIMENTS_DNAME/exp_id
        self.intermediate_dpath = self.experiment_dpath/self.DATA_DNAME/self.INTERMEDIATE_DNAME
        self.processed_dpath = self.experiment_dpath/self.DATA_DNAME/self.PROCESSED_DNAME
        self.output_dpath = self.experiment_dpath/self.OUTPUT_DNAME

        self.intermediate_dpath.rmtree(); self.intermediate_dpath.mkdir()
        self.processed_dpath.rmtree(); self.processed_dpath.mkdir()
        self.output_dpath.rmtree(); self.output_dpath.mkdir()

        self.raw_dpath = self.experiment_dpath/self.DATA_DNAME/self.RAW_DNAME

        for run_id in sorted([x.name for x in self.raw_dpath.iterdir() if x.is_dir()]):
            self.train_ann (
                experiment_dpath=self.experiment_dpath,
                latest_run_id=run_id
            )


    def train_nontrained (self, exp_id:str, data_built : bool) -> None:
        self.experiment_dpath = self.PACKAGE_DPATH/self.EXPERIMENTS_DNAME/exp_id
        self.intermediate_dpath = self.experiment_dpath/self.DATA_DNAME/self.INTERMEDIATE_DNAME
        self.processed_dpath = self.experiment_dpath/self.DATA_DNAME/self.PROCESSED_DNAME
        self.output_dpath = self.experiment_dpath/self.OUTPUT_DNAME
        self.raw_dpath = self.experiment_dpath/self.DATA_DNAME/self.RAW_DNAME
        self.latest_run_id = sorted([x.name for x in self.raw_dpath.iterdir() if x.is_dir()])[-1]
        experiment_id = self.experiment_dpath.name
        self.processed_fpath = self.processed_dpath/f"{experiment_id}{self.PROCESSED_FNAME_EXTENSION}"
        self.checkpoint_fpath = self.output_dpath/self.CHECKPOINT_FNAME

        run_ids = sorted([x.name for x in self.raw_dpath.iterdir() if x.is_dir()])
        if data_built: run_ids = [run_ids[-1]]
        
        for self.latest_run_id in run_ids:
            if self.latest_run_id not in [x.stem for x in self.intermediate_dpath.iterdir() if x.is_dir()]:

                print('\n\n.--- TRAIN FORGETFUL ANN\n|')
                print(f"|  Experiment: {experiment_id}")
                #self.createLink2Directory(self.raw_lpath, experiment_dpath, msg_prefix='|    - Raw data:          ')
                self.initDirectory(self.raw_dpath,          msg_prefix='|    - Raw data:          ')
                self.initDirectory(self.intermediate_dpath, msg_prefix='|    - Intermediate data: ')
                self.initDirectory(self.processed_dpath,    msg_prefix='|    - Processed data:    ')
                self.initDirectory(self.output_dpath,       msg_prefix='|    - Output data:       ')
                self.initCheckpoint(                        msg_prefix='|    - Checkpoint:        ')
                if not self.initRecordings(                 msg_prefix='|    - Recordings:        '): return
                    
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


        self.dataset = forgetful_dataset.ForgetfulDataset (
            df=self.df_processed,
            id=experiment_id,
            cnn_cols=self.config['data']['input']['cnn'],
            cat_cols=self.config['data']['input']['cat'],
            lbl_cols=self.config['data']['label'],
            msg_pfx='|  '
        )
        print('|')

        bs = self.config['learn']['batch_size']
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
        with tqdm(range(self.config['learn']['num_epochs']), desc='|  Epochs   ', position=0, ncols=NCOLS) as train_PBAR:
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
                    fname=self.USER_INPUT_FNAME,
                    log = log_savings,
                    pbar=train_PBAR,
                    msg_prefix=pfx
                )
                self.export_dict_as_json (
                    dictionary=self.config, 
                    fname=self.CONFIG_FNAME,
                    log = log_savings,
                    pbar=train_PBAR,
                    msg_prefix=pfx
                )
                self.export_dict_as_json (
                    dictionary=self.recordings, 
                    fname=self.RECORDINGS_FNAME,
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
                self.export_model_as_torch_script_module(
                    scripting_mode='annotated',
                    log = log_savings,
                    pbar=train_PBAR, 
                    msg_prefix=pfx
                )
                self.export_model_as_torch_script_module(
                    scripting_mode='traced',
                    log = log_savings,
                    pbar=train_PBAR, 
                    msg_prefix=pfx
                )
                log_savings = False

                train_PBAR.write(f"|    [Epoch {self.epoch_i:04d}]  Loss: {self.recordings['loss']['train'][self.num_prev_runs][-1]:.10f} (train) | {'None'} (valid)  //  LR:  {self.recordings['learn_rate'][self.num_prev_runs][-1]:.10f}")      

                if self.epoch_i + 1 == self.config['learn']['num_epochs']: train_PBAR.write("|", end='\n')
        print("|_________________________________________________")



    def roscb_train_ann (self, req:std_srvs.srv.EmptyRequest) -> std_srvs.srv.EmptyResponse:
        
        self.train_ann (
            experiment_dpath=pathlib.Path(rospy.get_param('EXPERIMENT_DPATH')),
            latest_run_id=rospy.get_param('LATEST_RUN_ID')
        )
        return std_srvs.srv.EmptyResponse()

    def train_ann (
        self,
        experiment_dpath:pathlib.Path,
        latest_run_id:str
    ) -> None:
        assert isinstance(experiment_dpath, pathlib.Path), 'Wrong type'
        assert isinstance(latest_run_id, str), 'Wrong type'

        self.latest_run_id = latest_run_id
        self.experiment_dpath = experiment_dpath
        experiment_id = experiment_dpath.name
        self.raw_dpath = experiment_dpath/self.DATA_DNAME/self.RAW_DNAME
        self.intermediate_dpath = experiment_dpath/self.DATA_DNAME/self.INTERMEDIATE_DNAME
        self.processed_dpath = experiment_dpath/self.DATA_DNAME/self.PROCESSED_DNAME
        self.processed_fpath = self.processed_dpath/f"{experiment_id}{self.PROCESSED_FNAME_EXTENSION}"
        self.output_dpath = experiment_dpath/self.OUTPUT_DNAME
        self.checkpoint_fpath = self.output_dpath/self.CHECKPOINT_FNAME

        print('\n\n.--- TRAIN FORGETFUL ANN\n|')
        print(f"|  Experiment: {experiment_id}")
        #self.createLink2Directory(self.raw_lpath, experiment_dpath, msg_prefix='|    - Raw data:          ')
        self.initDirectory(self.raw_dpath,          msg_prefix='|    - Raw data:          ')
        self.initDirectory(self.intermediate_dpath, msg_prefix='|    - Intermediate data: ')
        self.initDirectory(self.processed_dpath,    msg_prefix='|    - Processed data:    ')
        self.initDirectory(self.output_dpath,       msg_prefix='|    - Output data:       ')
        self.initCheckpoint(                        msg_prefix='|    - Checkpoint:        ')
        if not self.initRecordings(                 msg_prefix='|    - Recordings:        '): return
            

        

        print('|')
        if self.buildIntermediateData (runID=latest_run_id):
            print('|')
            self.buildProcessedData (RunID=latest_run_id)
            self.plotExpertInterventionShare()
        else:
            print('|')
            print('|  Processed Data:')
            self.loadProcessedData()
            print(f'|    - Loaded #{len(self.df_processed)} sequences')
        print('|')


        self.dataset = forgetful_dataset.ForgetfulDataset (
            df=self.df_processed,
            id=experiment_id,
            cnn_cols=self.config['data']['input']['cnn'],
            cat_cols=self.config['data']['input']['cat'],
            lbl_cols=self.config['data']['label'],
            msg_pfx='|  '
        )
        print('|')

        bs = self.config['learn']['batch_size']
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
        with tqdm(range(self.config['learn']['num_epochs']), desc='|  Epochs   ', position=0, ncols=NCOLS) as train_PBAR:
            pfx = '|    - '
            for self.epoch_i in train_PBAR:
                
                self.train_one_epoch ()
                
                self.export_dict_as_json (
                    dictionary=self.user_input, 
                    fname=self.USER_INPUT_FNAME,
                    log = log_savings,
                    pbar=train_PBAR,
                    msg_prefix=pfx
                )
                self.export_dict_as_json (
                    dictionary=self.config, 
                    fname=self.CONFIG_FNAME,
                    log = log_savings,
                    pbar=train_PBAR,
                    msg_prefix=pfx
                )
                self.export_dict_as_json (
                    dictionary=self.recordings, 
                    fname=self.RECORDINGS_FNAME,
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
                self.export_model_as_torch_script_module(
                    scripting_mode='annotated',
                    log = log_savings,
                    pbar=train_PBAR, 
                    msg_prefix=pfx
                )
                self.export_model_as_torch_script_module(
                    scripting_mode='traced',
                    log = log_savings,
                    pbar=train_PBAR, 
                    msg_prefix=pfx
                )
                log_savings = False

                train_PBAR.write(f"|    [Epoch {self.epoch_i:04d}]  Loss: {self.recordings['loss']['train'][self.num_prev_runs][-1]:.10f} (train) | {'None'} (valid)  //  LR:  {self.recordings['learn_rate'][self.num_prev_runs][-1]:.10f}")      

                if self.epoch_i + 1 == self.config['learn']['num_epochs']: train_PBAR.write("|", end='\n')
        print("|_________________________________________________")


    


    def export_model_as_torch_script_module (self, scripting_mode:str, log:bool=True, pbar:tqdm=None, msg_prefix:str='') -> None:
        if scripting_mode == 'annotated':
            fpath = self.output_dpath/self.ANNOTATED_FNAME
        elif scripting_mode == 'traced':
            fpath = self.output_dpath/self.TRACED_FNAME
        else:
            raise ValueError(f"Scripting mode: {scripting_mode} neither 'annotated' nor 'traced'")

        try: 
            if scripting_mode == 'annotated':
                self.model.exportAsAnnotatedTorchScriptModule(fpath)
            elif scripting_mode == 'traced':
                batch = next(iter(self.dataloader))
                self.model.exportAsTracedTorchScriptModule(
                    batch['input']['cnn'].to(TORCH_DEVICE),
                    batch['input']['cat'].to(TORCH_DEVICE),
                    fpath
                )
            msg = f"{msg_prefix}Export model to \"{self.rel(fpath)}\""
        
        except Exception as e:
            msg = f"{msg_prefix}Failed to export model to \"{self.rel(fpath)}\", error: {e}"
        
        if log: print(msg) if (pbar is None) else pbar.write(msg)


    def save_recordings_plots (self, log:bool=True, pbar:tqdm=None, msg_prefix:str='') -> None:        
        fpath1 = self.output_dpath/self.RECORDINGS_PLOT_LIN_FNAME 
        fpath2 = self.output_dpath/self.RECORDINGS_PLOT_LOG_FNAME  
        
        try:
            plt.rcParams['text.usetex'] = True
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                fig, ax1 = plt.subplots(figsize=(10, 7))
            color = 'tab:orange'
            ax1.set_xlabel('Epochs')
            ax1.set_ylabel(self.config['learn']['loss']['id'], color=color)

            epoch_cnt = 1
            for loss in self.recordings['loss']['train']:
                label = 'Training' if (epoch_cnt == 1) else None
                x_axis = list(range(epoch_cnt, epoch_cnt + len(loss)))
                epoch_cnt += len(loss)
                plt.plot(x_axis, loss, color=color, linestyle=':', label=label)
            
            epoch_cnt = 1
            for loss in self.recordings['loss']['valid']:
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
                for learn_rate in self.recordings['learn_rate']:
                    label = None if (epoch_cnt != 1) else\
                        f"{self.config['learn']['lr_scheduler']['id']}, $\gamma={self.config['learn']['lr_scheduler']['gamma']}$"
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

            msg1 = f"{msg_prefix}Save plot as \"{self.rel(fpath1)}\""
            msg2 = f"{msg_prefix}Save plot as \"{self.rel(fpath2)}\""

        except Exception as e:
            msg1 = f"{msg_prefix}Failed to save plot as \"{self.rel(fpath1)}\", error: {e}"
            msg2 = f"{msg_prefix}Failed to save plot as \"{self.rel(fpath2)}\", error: {e}"
        
        if log:
            print(msg1) if (pbar is None) else pbar.write(msg1)
            print(msg2) if (pbar is None) else pbar.write(msg2)
        

    def export_dict_as_json (self, dictionary:Dict, fname:str, log:bool=True, pbar:tqdm=None, msg_prefix:str='') -> None:
        fpath = self.output_dpath/fname
        
        try:
            with open(fpath, 'w') as file:
                file.write(json.dumps(dictionary, sort_keys=True, indent=4))
            msg = f"{msg_prefix}Export dict to \"{self.rel(fpath)}\""

        except Exception as e:
            msg = f"{msg_prefix}Failed to export dict to \"{self.rel(fpath)}\", error: {e}"
        
        if log: print(msg) if (pbar is None) else pbar.write(msg)




    #def roscb_brain_load_checkpoint (self, req: StringTriggerRequest) -> StringTriggerResponse:
    #    res = StringTriggerResponse()
    #    res.success = self.load_checkpoint(req.message)
    #    return res

    def load_checkpoint (self, fpath: pathlib.Path, log:bool=True) -> bool:
        try: 
            self.checkpoint = torch.load(fpath)

        except Exception as e: 
            print(f"Failed to load checkpoint from \"{fpath}\", error: {e}")
            return False

        self.model.load_state_dict(self.checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(self.checkpoint['optimizer_state_dict'])
        self.epoch_i = self.checkpoint['epoch']
        self.loss = self.checkpoint['loss']
        if log: print(f"Loaded checkpoint from \"{fpath}\"")
        
        for g in self.optimizer.param_groups:
            g['lr'] = self.config['learn']['lr_scheduler']['init']

        
        return True





    #def roscb_save_checkpoint (self, req: StringTriggerRequest) -> StringTriggerResponse:
    #    res = StringTriggerResponse()
    #    res.success = self.save_checkpoint(req.message)
    #    return res

    def save_checkpoint (self, fpath:pathlib.Path=None, log:bool=True, pbar:tqdm=None, msg_prefix:str='') -> bool:
        if fpath is None: fpath = self.checkpoint_fpath
        msg_body = f"checkpoint as \"{self.rel(fpath)}"

        try:
            torch.save({
                'epoch': self.epoch_i,
                'model_state_dict': self.model.state_dict(),
                'optimizer_state_dict': self.optimizer.state_dict(),
                'loss': self.loss,
                }, 
                fpath)
            msg = f"{msg_prefix}Save {msg_body}"
            success = True

        except Exception as e:
            msg = f"{msg_prefix}Failed to save {msg_body}, error: {e}"
            success = False
        
        if log: print(msg) if (pbar is None) else pbar.write(msg)
        return success
        
        

    




    def roscb_flightmare_rgb (self, msg: sensor_msgs.msg.Image) -> None:
        self.rgb_mgs = msg
        if INFER_EVERY_INCOMING_FRAME: self.roscb_infer_once()

    def roscb_ground_truth_imu (self, msg: sensor_msgs.msg.Imu) -> None:
        self.imu_msg = msg

    def roscb_infer_once (self):
        inference_t0 = rospy.Time.now ()

        try: 
            self.rgb = self.bridge.imgmsg_to_cv2(self.rgb_mgs, "bgr8")
            self.rgb_dt = self.rgb_mgs.header.stamp.to_sec() - self.rgb_t_last
            self.rgb_t_last = self.rgb_mgs.header.stamp.to_sec()
        except cv_bridge.CvBridgeError as e: 
            print(e)
            self.rgb = None
            return
        
        

        try: 
            self.imu = np.array(
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
            self.imu_dt = self.imu_msg.header.stamp.to_sec() - self.imu_t_last
            self.imu_t_last = self.imu_msg.header.stamp.to_sec()
        except Exception as e:
            print(e)
            self.imu = None
            return

                
        C = self.config['data']['processed']['rgb']['num_channels']
        H = self.config['data']['processed']['rgb']['height']
        W = self.config['data']['processed']['rgb']['width']
        
        rgb = self.preprocessRGB(self.rgb)
        rgb = np.array(rgb / 255.0, dtype=np.float32) 
        rgb = np.transpose(rgb, (2, 0, 1)) # channel size to index 0
        rgb = torch.tensor(rgb, dtype=torch.float32)
        rgb = rgb.view(1, 1, C, H, W).to(TORCH_DEVICE)


        cat = np.concatenate(
            (
                np.array([self.rgb_dt, self.imu_dt]), 
                self.imu,
                np.array([self.max_speed])
            ), 
            dtype=np.float32
        )[self.cat_mask]
        cat = torch.tensor(cat, dtype=torch.float32)
        cat = cat.view(1, 1, len(self.config['data']['input']['cat'])).to(TORCH_DEVICE)

        inference_t1 = rospy.Time.now() - inference_t0

        output, self.h = self.model.forward(
            x_img=rgb, 
            x_cat=cat,
            h=self.h.data
        )
        self.output = output[0].detach().cpu().numpy()

        inference_t2 = rospy.Time.now() - inference_t1 - inference_t0

        rospy.loginfo_throttle(
            5.0, 
            f"[{rospy.get_name()}]    Inference time [ms]:  - processing {1000*inference_t1.to_sec()}  - forwarding {1000*inference_t2.to_sec()}"
        )

        self.publishOutput ()
        


    def publishOutput (self) -> None:
        
        self.rospub_brain_output.publish(
            geometry_msgs.msg.Point(
                x=self.output[0],
                y=self.output[1],
                z=self.output[2]
            )
        )
        

    def roscb_timer (self, te: rospy.timer.TimerEvent) -> None:
        if te.last_duration is not None:
            if rospy.Duration(te.last_duration) > self.period:
                rospy.logwarn(f"[{rospy.get_name()}]    ROS timer took {te.last_duration} > {self.period} s.")
        
        self.roscb_infer_once()


  

if __name__ == '__main__':
    rospy.init_node('forgetful_brain')
    
    
    rospy.set_param('SIM_UNITY_DRONE_CAMERA_WIDTH', 720)
    rospy.set_param('SIM_UNITY_DRONE_CAMERA_HEIGHT', 480)
    rospy.set_param('DRONE_MAIN_LOOP_FREQUENCY', 50.0)

    dagger = DAGGER (user_input.user_input)
    dagger.train_nontrained(exp_id='UTC_2022_7_1_7_27_56___SEQLEN_x', data_built=False)
    #dagger.train_ann(
    #    experiment_dpath=pathlib.Path('/home/fm/catkin_ws/src/forgetful_drone/experiments/UTC_2022_6_13_20_21_51'),
    #    latest_run_id='0002___0_0_0_1_0_1_04.00_00.70___002'
    #)
    rospy.spin()