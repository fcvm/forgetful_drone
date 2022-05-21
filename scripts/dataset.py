
import torch
import cv2
import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

import config
#import utils
from torch.utils.data import Dataset, DataLoader
from tqdm import tqdm
import random
import pathlib
from itertools import compress

from typing import Any, List
from typing import Dict
import json

import copy






def check_number (what: str, actual_num: int, control_num: int) -> None:
    if actual_num != control_num:
        raise AssertionError(f"# {what}: {actual_num} (actual number) != {control_num} (control number).")





class ForgetfulDataset (Dataset):
    
    def __init__ (
        self, 
        df: pd.DataFrame,
        id: str,
        params: Dict[str, Dict[str, Any]],
        *args, 
        **kwargs
    ) -> None:
        super().__init__(*args, **kwargs)
        self.df = df
        self.seq_len = len(df[0])
        self.id = id
        self.params = params
       
        


    def __len__ (self):
        return len(self.df)
    
    def __getitem__(self, idx: int):
        
        # Sequence of samples
        df_seq = self.df[idx]

        # Image
        C = self.params['image']['num_channels']
        H = self.params['image']['height']
        W = self.params['image']['width']
        # In PyTorch, images are represented as [channels, height, width]
        img_seq = np.zeros((self.seq_len, C, H, W), dtype=np.float32)
        for i, img_fpath in df_seq[self.params['columns']['image']].iterrows():
            img = cv2.imread(img_fpath[0])
            #height, width, channels = img.shape
            img = cv2.resize(img, (W, H))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = img / 255.0
            img = np.transpose(img, (2, 0, 1)) # Channel size to index 0
            img_seq[i] = img
        
        # IMU
        imu_seq = np.array(
            df_seq[self.params['columns']['imu']], dtype=np.float32
        )

        # Label
        label = np.array(
            df_seq[self.params['columns']['label']].iloc[-1], dtype=np.float32
        )

        return {
            'inputs': {
                'image': torch.tensor(img_seq, dtype=torch.float),
                'imu': torch.tensor(imu_seq, dtype=torch.float)
            },
            'label': torch.tensor(label, dtype=torch.float)
        }

    def plot_samples (self):
        pass














class ForgetfulData:
    
    def __init__(
        self,
        data_dpath: pathlib.Path,
        batch_size: int,
        params: Dict[str, Any]
    ) -> None:
        self.data_dpath = data_dpath
        self.batch_size = batch_size
        self.params = params
        self.setPaths()
        self.setProcessedRunFpaths()
        if not (self.processedRunFilesExist() and self.ProcessedRunFilesIntact()):
            self.generateProcessedRunFiles()
        self.setRunFilesDict()
        self.setTrainAndValidRunFiles()
        self.logInfo()

        self.setTrainingDatasetAndLoader()
        print('|')
        self.setValidationDatasetsAndLoaders()
        print("|_________________________________________________")


    def setPaths (self) -> None:
        self.raw_dpath = self.data_dpath/self.params['raw']['dirname']
        if not self.raw_dpath.is_dir():
            raise AssertionError(f"'{self.raw_dpath}' is not a directory")
        self.processed_dpath = self.data_dpath/self.params['processed']['dirname']
        if not self.processed_dpath.is_dir():
            raise AssertionError(f"'{self.processed_dpath}' is not a directory")



    def setProcessedRunFpaths (self) -> None:
        self.run_fpaths = {
            'processed': sorted(
                [x for x in self.processed_dpath.iterdir()\
                    if x.is_file()\
                    and x.stem[:4] == self.params['processed']['filename']['prefix']\
                    and x.suffix == self.params['processed']['filename']['extension']])
        }

    def processedRunFilesExist (self) -> bool:
        return len(self.run_fpaths['processed']) > 0

    def ProcessedRunFilesIntact (self) -> bool:
        fp = random.choice(self.run_fpaths['processed'])
        df = pd.read_csv(fp, usecols=self.params['processed']['columns']['image'])
        img_fpath = pathlib.Path(df.iloc[0][0])
        return img_fpath.is_file()


    def generateProcessedRunFiles (self) -> List[pathlib.Path]:
        dp = self.data_dpath/self.params['raw']['dirname']
        racetrack_type_dpaths = sorted([x for x in self.raw_dpath.iterdir() if x.is_dir()])

        with tqdm(racetrack_type_dpaths) as pbar_rt:
            pbar_rt.set_description(f"GENERATE RUN FILES")
            for rt_dpath in pbar_rt:
                
                run_dpaths = sorted([x for x in rt_dpath.iterdir() if x.is_dir()])

                with tqdm(run_dpaths, leave=False) as pbar_runs:
                    pbar_runs.set_description(rt_dpath.name.upper().replace('_', ' '))
                    for run_dpath in pbar_runs:
                        pbar_runs.set_postfix_str(run_dpath.name)

                        img_dpath = run_dpath/self.params['raw']['run_dir_file_names']['image']
                        imu_fpath = run_dpath/self.params['raw']['run_dir_file_names']['imu']
                        wps_fpath = run_dpath/self.params['raw']['run_dir_file_names']['waypoint_speed']
                        cmd_fpath = run_dpath/self.params['raw']['run_dir_file_names']['control_command']
                        
                        img_df = pd.DataFrame(sorted([x for x in img_dpath.iterdir() if x.is_file()]))
                        imu_df = pd.read_csv(imu_fpath, delimiter=';', header=None)
                        wps_df = pd.read_csv(wps_fpath, delimiter=';', header=None)
                        cmd_df = pd.read_csv(cmd_fpath, delimiter=';', header=None)

                        img_df.columns = self.params['processed']['columns']['image']
                        imu_df.columns = self.params['processed']['columns']['imu']
                        wps_df.columns = self.params['processed']['columns']['waypoint_speed']
                        cmd_df.columns = self.params['processed']['columns']['control_command']

                        img_n = len(img_df)
                        imu_n = imu_df.shape[0]
                        wps_n = wps_df.shape[0]
                        cmd_n = cmd_df.shape[0]
                        assert img_n == imu_n == wps_n == cmd_n, 'Unequal amount of samples:\n'\
                            + f"\t# image: {img_n}\n\t# imu: {imu_n}\n\t# waypoint-speed: {wps_n}\n\t# control command: {cmd_n}"

                        run_id_df = pd.DataFrame([run_dpath.name] * img_n, columns=self.params['processed']['columns']['run'])

                        run_df = pd.concat(
                            [run_id_df, img_df, imu_df, wps_df, cmd_df], axis=1)

                        run_fpath = self.processed_dpath/(run_dpath.name\
                            + self.params['processed']['filename']['extension'])
                        run_df.to_csv(run_fpath, index=False)

        self.setProcessedRunFpaths()


    def setRunFilesDict (self) -> None:

        racetrack_types: List[str] = self.params['raw']['racetrack_types']
        unity_scenes: List[str] = self.params['raw']['unity_scenes']
        
        num_sites: int = len(self.params['raw']['scene_sites'])
        num_gates: int = len(self.params['raw']['gate_types'])
        
        num_reps: Dict[str, int] = self.params['raw']['repetitions']
        gaps: Dict[str, List[None or str]] = self.params['raw']['gap_types']
        dirs: Dict[str, List[None or str]] = self.params['raw']['directions']



        # control numbers of run files
        cnums = dict()
        cnums['total'] = 0
        for racetrack_type in racetrack_types:
            cnums[racetrack_type] = dict()
            cnums[racetrack_type]['per_scene'] = dict()
            cnums[racetrack_type]['per_scene']['per_site'] = num_gates\
                                                    * num_reps[racetrack_type]\
                                                    * len(gaps[racetrack_type])\
                                                    * len(dirs[racetrack_type])
            cnums[racetrack_type]['per_scene']['total'] = num_sites * cnums[racetrack_type]['per_scene']['per_site']
            cnums[racetrack_type]['total'] = cnums[racetrack_type]['per_scene']['total'] * len(unity_scenes)
            cnums['total'] += cnums[racetrack_type]['total']


        # paths of run files
        rfps = dict()
        rfps['total'] = self.run_fpaths['processed']
        check_number('run files', len(rfps['total']), cnums['total'])

        # Sort run files by racetrack type
        for racetrack_type in racetrack_types:
            rfps[racetrack_type] = dict()
            rfps[racetrack_type]['total']\
                = self.params['processed']['filename']['identifier'][racetrack_type](rfps['total'])
            check_number(f'{racetrack_type} - run files', 
                len(rfps[racetrack_type]['total']), cnums[racetrack_type]['total'])
            
        # Sort racetrack type run files by unity scene
        for racetrack_type in racetrack_types:
            for i, unity_scene in enumerate(unity_scenes):
                rfps[racetrack_type][unity_scene]\
                    = [x for x in rfps[racetrack_type]['total'] if int(x.stem[4]) == i]
                check_number(f'{racetrack_type} - {unity_scene} - run files', 
                    len(rfps[racetrack_type][unity_scene]), cnums[racetrack_type]['per_scene']['total'])

        # Generate masks to sort runs into training and validation
        masks = dict()
        for racetrack_type in racetrack_types:
            masks[racetrack_type] = dict()
            masks[racetrack_type]['total'] = dict()
            masks[racetrack_type]['per_scene'] = dict()
            if racetrack_type == racetrack_types[0]:
                masks[racetrack_type]['per_scene']['train']\
                    = ([True] * (num_reps['figure_8_racetrack'] - 1) + [False])\
                    * int(cnums[racetrack_type]['per_scene']['total'] / num_reps['figure_8_racetrack'])
            elif racetrack_type == racetrack_types[1]:
                num_valid_runs = [round(cnums[racetrack_type]['per_scene']['per_site'] / num_sites)] * num_sites
                num_valid_runs[-1] -= num_valid_runs[0] * num_sites - cnums[racetrack_type]['per_scene']['per_site']
                masks[racetrack_type]['per_scene']['train'] = [True] * cnums[racetrack_type]['per_scene']['total']
                for i in range(num_sites):
                    start_idx = i * cnums[racetrack_type]['per_scene']['per_site'] + i * num_valid_runs[0]
                    end_idx = start_idx + num_valid_runs[i]
                    masks[racetrack_type]['per_scene']['train'][start_idx:end_idx] = [False] * num_valid_runs[i]

            masks[racetrack_type]['per_scene']['valid'] = [not x for x in masks[racetrack_type]['per_scene']['train']]


        # Sort run files into train/valid
        run_files_dict = dict()
        for racetrack_type in racetrack_types:
            run_files_dict[racetrack_type] = dict()
            for unity_scene in unity_scenes:
                run_files_dict[racetrack_type][unity_scene] = dict()
                run_files_dict[racetrack_type][unity_scene]['train']\
                    = list(compress(rfps[racetrack_type][unity_scene], masks[racetrack_type]['per_scene']['train']))
                run_files_dict[racetrack_type][unity_scene]['valid']\
                    = list(compress(rfps[racetrack_type][unity_scene], masks[racetrack_type]['per_scene']['valid']))


        self.run_files_dict = run_files_dict


    def printRunFilesDict (self) -> None:
        rfd_serializable = copy.deepcopy(self.run_files_dict)
        for key0 in rfd_serializable.keys():
            for key1 in rfd_serializable[key0].keys():
                for key2 in rfd_serializable[key0][key1].keys():
                    rfd_serializable[key0][key1][key2] = [str(x) for x in rfd_serializable[key0][key1][key2]]
        print(json.dumps(rfd_serializable, sort_keys=True, indent=4))


    def setTrainAndValidRunFiles (self) -> None:
        run_files_train = []
        run_files_valid = []
        for rt in self.params['racetrack_types']:
            mode = 'train'
            for us in self.params['unity_scenes'][mode]:
                run_files_train += self.run_files_dict[rt][us][mode]
            mode = 'valid'
            for us in self.params['unity_scenes'][mode]:
                run_files_valid += self.run_files_dict[rt][us][mode]
        random.shuffle(run_files_train)
        self.run_fpaths['loaded'] = {
            'train': run_files_train,
            'valid': run_files_valid
        }

    def printTrainAndValidRuns (self) -> None:
        runs = {
            'train': [x.stem for x in self.run_fpaths['loaded']['train']],
            'valid': [x.stem for x in self.run_fpaths['loaded']['valid']]
        }
        print('Loaded run files:')
        print(json.dumps(runs, sort_keys=True, indent=4))


    def logInfo (self) -> None:
        print("")
        print("")
        print(".--- DATA LOADERS")
        print("|")
        print(f"|  - Data")
        print(f"|     - Racetrack types: {self.params['racetrack_types']}")
        print(f"|     - Unity Scenes")
        print(f"|        - Training: {self.params['unity_scenes']['train']}")
        print(f"|        - Validation: {self.params['unity_scenes']['valid']}")
        print(f"|  - Number of runs")
        print(f"|     - Training: {len(self.run_fpaths['loaded']['train'])}")
        print(f"|     - Validation: {len(self.run_fpaths['loaded']['valid'])}")
        print(f"|  - Number of samples per run: {self.params['num_samples_per_run'] if self.params['num_samples_per_run'] != None else 'all'}")
        print(f"|  - Image resize factor: {self.params['image']['resize_factor']}")
        print(f"|  - Sequential length/step: {self.params['sequential']['length']}/{self.params['sequential']['step']}")
        print(f"|  - Inputs: {self.params['inputs']}")
        print(f"|  - Label: {self.params['label']}")
        print("|")



    def getRunDataset (
        self,
        run_fpath: pathlib.Path,
        for_train: bool
    ) -> ForgetfulDataset:

        # create dataframe according to spec. inputs and label
        input_label_cols = []
        for inputt in [*self.params['inputs'], 'label']:
            input_label_cols += self.params['loaded']['columns'][inputt]        
        df = pd.read_csv(
            run_fpath,
            usecols=input_label_cols
        )

        # Limit number of samples per run, if spec.
        if not self.params['num_samples_per_run'] == None:
            num_samples = len(df)
            random_idx = random.randint(0, num_samples - self.params['num_samples_per_run'])
            df = df[random_idx : random_idx + self.params['num_samples_per_run']]
            df = df.reset_index(drop=True)
        
        # Train on sequences, validate on incoming samples (real time)
        if for_train:
            seq_len = self.params['sequential']['length']
            seq_step = self.params['sequential']['step']
            final_idx = len(df) - seq_len
            start_idx = random.randrange(1 + final_idx % seq_step)
        else:
            seq_len = 1
            seq_step = 1
            final_idx = len(df) - seq_len
            start_idx = 0
        df_seq = []
        for i in range(start_idx, final_idx, seq_step):
            df_seq.append(df[i : i + seq_len].reset_index(drop=True))

        return ForgetfulDataset(
            df_seq, 
            run_fpath.stem,
            self.params['loaded']
        )


    def setTrainingDatasetAndLoader (self) -> None:

        runs = self.run_fpaths['loaded']['train']
        run_datasets: List[ForgetfulDataset] = []

        with tqdm(enumerate(runs)) as pbar:
            pbar.set_description(f"|  Training data loader")
            for i, run_fpath in pbar:
                run_id = run_fpath.stem
                pbar.set_postfix_str(run_id)

                if i % 4 == 0: 
                    pbar.write(f"|    {run_id}", end='')
                elif i % 4 == 3: 
                    pbar.write(f"    {run_id}", end='\n')
                else: 
                    pbar.write(f"    {run_id}", end='')
                
                if i + 1 == len(runs) and i % 4 != 3: 
                    pbar.write("", end='\n')

                run_datasets.append(self.getRunDataset(run_fpath, for_train=True))

        dataset = run_datasets[0]
        dfs = [x.df for x in run_datasets]
        dataset.df = [sequence for df in dfs for sequence in df]
        self.datasets = {
            'train': {
                'merged': dataset
            }
        }

        self.data_loaders = {
            'train': {
                'merged': DataLoader(
                    self.datasets['train']['merged'],
                    batch_size=self.batch_size,
                    shuffle=True,
                    drop_last=True
                )
            }
        }


    def setValidationDatasetsAndLoaders (self) -> None:

        runs = self.run_fpaths['loaded']['valid']
        self.datasets['valid'] = {}
        self.data_loaders['valid'] = {}
        
        with tqdm(enumerate(runs)) as pbar:
            pbar.set_description(f"|  Validation data loaders")
            for i, run_fpath in pbar:
                run_id = run_fpath.stem
                pbar.set_postfix_str(run_id)

                if i % 4 == 0: pbar.write(f"|    {run_id}", end='')
                elif i % 4 == 3: pbar.write(f"    {run_id}", end='\n')
                else: pbar.write(f"    {run_id}", end='')
                if i + 1 == len(runs) and i % 4 != 3: 
                    pbar.write("", end='\n')
                    
                self.datasets['valid'][run_id]\
                    = self.getRunDataset(run_fpath, for_train=False)
                
                self.data_loaders['valid'][run_id] = DataLoader(
                    self.datasets['valid'][run_id],
                    batch_size=1, 
                    shuffle=False,
                    drop_last=True
                )








def get_data_loaders() -> Dict[str, Dict[str, DataLoader]]:
    params = config.get_configuration(config.user_input)
    fd = ForgetfulData(
        config.get_root_dpath()/params['data']['dirname'],
        params['learn']['batch_size'],
        params['data']
    )
    return fd.data_loaders



















if __name__ == '__main__':
    
    params = config.get_configuration()

    dataloaders = ForgetfulData(
        config.get_root_dpath()/params['data']['dirname'],
        params['learn']['batch_size'],
        params['data']
    )
    #dataloaders.printRunFilesDict()
    #dataloaders.printTrainAndValidRuns()

    dataloaders.datasets['train']['merged'][0]
    #print(dataloaders.data_loaders['train']['merged'])

























































'''def run_files_exist():
    return 0 < len(RUN_FILES_AVAILABLE)'''

'''def run_files_paths_exist():
    random_run_file = random.choice(RUN_FILES_AVAILABLE)
    dataset = pd.read_csv(random_run_file, usecols=config.COLS_INPUT_IMAGE_FILE)
    img_file = pathlib.Path(dataset.iloc[0][0])
    return img_file.is_file()'''

'''def generate_run_files():
    DATA_DIR = config.INPUT_DIR/"data"
    run_type_dirs_available = sorted([x for x in DATA_DIR.iterdir() if x.is_dir()])

    with tqdm(enumerate(run_type_dirs_available)) as TQDM:
        for run_type_dir_i, run_type_dir in TQDM:
            TQDM.set_description(f"Run Types")
            TQDM.set_postfix({'Type': run_type_dir.name})

            run_dirs = sorted([x for x in run_type_dir.iterdir() if x.is_dir()])

            with tqdm(enumerate(run_dirs), leave=False) as TQDM:
                for run_dir_i, run_dir in TQDM:
                    TQDM.set_description(f"Runs")
                    TQDM.set_postfix({'Run': run_dir.name})

                    img_dir = run_dir/"images"
                    imu_file = run_dir/"imu.txt"
                    std_labels_file = run_dir/"labels.txt"
                    ctrlcmd_labels_file = run_dir/"control_command.txt"
                    run_file = config.INPUT_DIR/(run_dir.name + ".csv")


                    img_files = sorted([x for x in img_dir.iterdir() if x.is_file()])
                    img_files = pd.DataFrame(img_files, columns = config.COLS_RUN_FILE_IMAGE)

                    imu_features = pd.read_csv(imu_file, delimiter=';', header=None)
                    imu_features.columns=config.COLS_RUN_FILE_IMU

                    standard_labels = pd.read_csv(std_labels_file, delimiter=';', header=None)
                    standard_labels.columns = config.COLS_RUN_FILE_STANDARD_LABEL

                    ctrlcmd_labels = pd.read_csv(ctrlcmd_labels_file, delimiter=';', header=None)
                    ctrlcmd_labels.columns = config.COLS_RUN_FILE_CONTROL_COMMAND

                    sample_n = len(img_files)
                    assert imu_features.shape[0] == sample_n, f"Mismatching number of samples: images [{sample_n}] != imu data [{imu_features.shape[0]}]"
                    assert standard_labels.shape[0] == sample_n, f"Mismatching number of samples: images [{sample_n}] != standard labels [{standard_labels.shape[0]}]"
                    assert ctrlcmd_labels.shape[0] == sample_n, f"Mismatching number of samples: images [{sample_n}] != control command labels [{ctrlcmd_labels.shape[0]}]"
    
                    run_ids = pd.DataFrame([run_dir.name] * sample_n, columns = ['run_id'])

                    run_samples = pd.concat(
                        [
                            run_ids,
                            img_files,
                            imu_features, 
                            standard_labels, 
                            ctrlcmd_labels
                        ], 
                        axis=1
                    )

                    run_samples.to_csv(run_file, index=False)'''

'''def get_run_files_dict():

    _RUN_TYPES = config.RUN_FILES_RUN_TYPES
    _RUN_TYPE_N = len(_RUN_TYPES)
    _SCENES = config.RUN_FILES_SCENES
    _SCENE_N = len(config.RUN_FILES_SCENES)
    _SITE_N = config.RUN_FILES_SITE_N
    _GATE_N = config.RUN_FILES_GATE_N
    _FIG8_REP_N = config.RUN_FILES_FIG8_REPETITION_N
    _ITL_CURV_N = config.RUN_FILES_ITL_CURVATURE_N
    _ITL_DIREC_N = config.RUN_FILES_ITL_DIRECTION_N
    _NAME_PREFIX = config.RUN_FILES_NAME_PREFIX
    _EXTENSION = config.RUN_FILES_EXTENSION


    run_n = dict()
    run_n['all'] = 0
    for run_type in _RUN_TYPES:
        run_n[run_type] = dict()
        run_n[run_type]['scene'] = dict()
        if run_type == _RUN_TYPES[0]:
            run_n[run_type]['scene']['site'] = _GATE_N * _FIG8_REP_N
        elif run_type == _RUN_TYPES[1]:
            run_n[run_type]['scene']['site'] = _GATE_N * _ITL_CURV_N * _ITL_DIREC_N
        run_n[run_type]['scene']['all'] = _SITE_N * run_n[run_type]['scene']['site']
        run_n[run_type]['all'] = run_n[run_type]['scene']['all'] * _SCENE_N
        run_n['all'] += run_n[run_type]['all']



    run_files = dict()
    run_files['all'] = sorted([
        x for x in config.INPUT_DIR.iterdir()\
            if x.is_file()\
                and x.name[:len(_NAME_PREFIX)] == _NAME_PREFIX\
                and x.name[-len(_EXTENSION):] == _EXTENSION
    ])  
    if len(run_files['all']) != run_n['all']:
        raise AssertionError(f"`len(run_files['all'])`  =  {len(run_files['all'])}  !=  {run_n['all']}")
    

    for run_type in _RUN_TYPES:
        run_files[run_type] = dict()
        if run_type == _RUN_TYPES[0]:
            run_files[run_type]['all'] = [x for x in run_files['all'] if not 'X' in x.name]
        elif run_type == _RUN_TYPES[1]:
            run_files[run_type]['all'] = [x for x in run_files['all'] if 'X' in x.name]
        if len(run_files[run_type]['all']) != run_n[run_type]['all']:
            raise AssertionError(f"{len(run_files[run_type]['all'])}  !=  {run_n[run_type]['all']}")


    for run_type in _RUN_TYPES:
        for scene_i, scene in enumerate(_SCENES):
            run_files[run_type][scene]\
                = [x for x in run_files[run_type]['all'] if x.name[:5] == "run_" + str(scene_i)]
            if len(run_files[run_type][scene]) != run_n[run_type]['scene']['all']:
                raise AssertionError(f"{len(run_files[run_type][scene])}  !=  {run_n[run_type]['scene']['all']}")


    masks = dict()
    for run_type in _RUN_TYPES:
        masks[run_type] = dict()
        masks[run_type]['all'] = dict()
        masks[run_type]['scene'] = dict()
        if run_type == _RUN_TYPES[0]:
            masks[run_type]['scene']['train'] = ([True] * (_FIG8_REP_N - 1) + [False]) * int(run_n[run_type]['scene']['all'] / _FIG8_REP_N)
        elif run_type == _RUN_TYPES[1]:
            valid_run_n = [round(run_n[run_type]['scene']['site'] / _SITE_N)] * _SITE_N
            valid_run_n[-1] -= valid_run_n[0] * _SITE_N - run_n[run_type]['scene']['site']
            masks[run_type]['scene']['train'] = [True] * run_n[run_type]['scene']['all']
            for site_i in range(_SITE_N):
                start_idx = site_i * run_n[run_type]['scene']['site'] + site_i * valid_run_n[0]
                end_idx = start_idx + valid_run_n[site_i]
                masks[run_type]['scene']['train'][start_idx:end_idx] = [False] * valid_run_n[site_i]
        masks[run_type]['scene']['valid'] = [not x for x in masks[run_type]['scene']['train']]



    RUN_FILES = dict()
    for run_type in _RUN_TYPES:
        RUN_FILES[run_type] = dict()
        for scene in _SCENES:
            RUN_FILES[run_type][scene] = dict()
            RUN_FILES[run_type][scene]['train']\
                = list(compress(run_files[run_type][scene], masks[run_type]['scene']['train']))
            RUN_FILES[run_type][scene]['valid']\
                = list(compress(run_files[run_type][scene], masks[run_type]['scene']['valid']))


    return RUN_FILES'''

# No assertion, overfit single random run
"""def get_run_files():

    run_files_dict = get_run_files_dict()

    '''
    # If specified, sort out intermediate target loss runs
    if config.RUN_USE_ITL:
        run_files_considered = RUN_FILES_AVAILABLE
    else:
        run_files_considered = [x for x in RUN_FILES_AVAILABLE if 'X' not in x.name]
    '''

    if config.RUN_USE_FIG8 and config.RUN_USE_ITL:
        run_types = config.RUN_FILES_RUN_TYPES
    elif config.RUN_USE_FIG8 and not config.RUN_USE_ITL:
        run_types = [config.RUN_FILES_RUN_TYPES[0]]
    elif not config.RUN_USE_FIG8 and config.RUN_USE_ITL:
        run_types = [config.RUN_FILES_RUN_TYPES[1]]
    else:
        raise ValueError(f"At least one run type must be activated.")
        
    # Choose runs according to mode

    if config.RUN_SELECTION_MODE == 'OVERFIT_SINGLE_RANDOM_RUN':
        
        run_type = random.choice(run_types)
        scene = random.choice(list(run_files_dict[run_type]))
        train_or_valid = random.choice(list(run_files_dict[run_type][scene]))
        run_files = [random.choice(list(run_files_dict[run_type][scene][train_or_valid]))]

        '''
        run_files = [random.choice(run_files_considered)]
        '''

        run_train_files = run_files
        run_valid_files = run_files

    elif config.RUN_SELECTION_MODE[:28] == 'TRAIN_AND_VALIDATE_ON_SCENE_':
        '''
        if not type(config.SCENE_VALID_RUN_N) == int:
            raise ValueError(f"'config.VALID_RUN_N_PER_SCENE={config.SCENE_VALID_RUN_N}' must be int for this mode.")

        scenes = config.RUN_SELECTION_MODE.split('_')[-1]
        scenes = [int(x) for x in scenes]
        '''
        scene_idxs = config.RUN_SELECTION_MODE.split('_')[-1]
        scene_idxs = [int(x) - 1 for x in scene_idxs]
        scenes = [config.RUN_FILES_SCENES[x] for x in scene_idxs]
        
        run_train_files = []
        run_valid_files = []

        for run_type in run_types:
            for scene in scenes:
                run_train_files += run_files_dict[run_type][scene]['train']
                run_valid_files += run_files_dict[run_type][scene]['valid']

        '''for scene_i in scenes:
            scene_run_files = [x for x in run_files_considered if str(x.name)[:5] == f"run_{scene_i}"]
            
            # Randomly take validation runs 
            random.shuffle(scene_run_files) 
            if 1 <= config.SCENE_VALID_RUN_N and config.SCENE_VALID_RUN_N < len(scene_run_files):
                scene_run_train_files = scene_run_files[config.SCENE_VALID_RUN_N:]
                scene_run_valid_files = scene_run_files[:config.SCENE_VALID_RUN_N]
            else:
                raise ValueError(f"'config.VALID_RUN_N_PER_SCENE={config.SCENE_VALID_RUN_N}' is not in {{1, ..., {len(scene_run_files)}}}.")
            
            # Randomly take training runs from remaining
            if not config.SCENE_TRAIN_RUN_N == 'ALL':
                if 1 <= config.SCENE_TRAIN_RUN_N and config.SCENE_TRAIN_RUN_N <= len(scene_run_train_files):
                    scene_run_train_files = random.sample(scene_run_train_files, config.SCENE_TRAIN_RUN_N)
                else: 
                    raise ValueError(f"'config.TRAIN_RUN_N_PER_SCENE={config.SCENE_TRAIN_RUN_N}' is not in {{1, ..., {len(scene_run_train_files)}}}.")

            run_train_files += scene_run_train_files
            run_valid_files += scene_run_valid_files'''


    elif config.RUN_SELECTION_MODE.split('_')[:3] == ['TRAIN', 'ON', 'SCENE']\
        and config.RUN_SELECTION_MODE.split('_')[4:8] == ['AND', 'VALIDATE', 'ON', 'SCENE']:
        
        scene_idxs_train = config.RUN_SELECTION_MODE.split('_')[3]
        scene_idxs_valid = config.RUN_SELECTION_MODE.split('_')[8]

        scene_idxs_train = [int(x) for x in scene_idxs_train]
        scene_idxs_valid = [int(x) for x in scene_idxs_valid]
        
        scenes_train = [config.RUN_FILES_SCENES[x] for x in scene_idxs_train]
        scenes_valid = [config.RUN_FILES_SCENES[x] for x in scene_idxs_valid]
        
        run_train_files = []
        run_valid_files = []

        for run_type in run_types:
            for scene in scenes_train:
                run_train_files += run_files_dict[run_type][scene]['train']
            for scene in scenes_valid:
                run_valid_files += run_files_dict[run_type][scene]['valid']



        '''scenes_train = config.RUN_SELECTION_MODE.split('_')[3]
        scenes_valid = config.RUN_SELECTION_MODE.split('_')[8]
        
        scenes_train = [int(x) for x in scenes_train]
        scenes_valid = [int(x) for x in scenes_valid]

        run_train_files = []
        run_valid_files = []

        for train_scene_i in scenes_train:
            scene_run_train_files = [x for x in run_files_considered if str(x.name)[:5] == f"run_{train_scene_i}"]
            if not config.SCENE_TRAIN_RUN_N == 'ALL':
                if 1 <= config.SCENE_TRAIN_RUN_N and config.SCENE_TRAIN_RUN_N <= len(scene_run_train_files):
                    scene_run_train_files = random.sample(scene_run_train_files, config.SCENE_TRAIN_RUN_N)
                else: 
                    raise ValueError(f"'config.TRAIN_RUN_N_PER_SCENE={config.SCENE_TRAIN_RUN_N}' is not in {{1, ..., {len(scene_run_train_files)}}}.")
            run_train_files += scene_run_train_files
            
        for valid_scene_i in scenes_valid:
            scene_run_valid_files = [x for x in run_files_considered if str(x.name)[:5] == f"run_{valid_scene_i}"]
            if not config.SCENE_VALID_RUN_N == 'ALL':
                if 1 <= config.SCENE_VALID_RUN_N and config.SCENE_VALID_RUN_N <= len(scene_run_valid_files):
                    scene_run_valid_files = random.sample(scene_run_valid_files, config.SCENE_VALID_RUN_N)
                else:
                    raise ValueError(f"'config.VALID_RUN_N_PER_SCENE={config.SCENE_VALID_RUN_N}' is not in {{1, ..., {len(scene_run_valid_files)}}}.")
            run_valid_files += scene_run_valid_files'''
    
    else:
        raise ValueError(f"No implementation provided for 'MODE={config.RUN_SELECTION_MODE}'.")


    random.shuffle(run_train_files)
    return run_train_files, run_valid_files"""








"""class ForgetfulDataset_seq(Dataset):
    def __init__(self, sequences, name):
        self.data = sequences
        self.name = name
    def __len__(self):
        return len(self.data)
    def __getitem__(self, i):
        pass
    def plot_samples(self):
        pass


class ForgetfulDataset_seq_Img2Wp(ForgetfulDataset_seq):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):
        seq = self.data[i]#.reset_index(drop=True)
        seq_l = len(seq)
            
        # --- IMAGE ---
        img_seq = np.zeros((seq_l, 3, config.IMAGE_HEIGHT_RESIZED, config.IMAGE_WIDTH_RESIZED), dtype=np.float32)

        for img_i, row in seq[config.COLS_INPUT_IMAGE_FILE].iterrows():
            img_file_path = row[0]
            img = cv2.imread(img_file_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
            img = img / 255.0
            img = np.transpose(img, (2, 0, 1)) # Channel size to index 0
            img_seq[img_i] = img

        # --- IMU ---
        #imu = self.samples[config.COLS_INPUT_IMU].iloc[i]
        #imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        wp = seq[config.COLS_LABELS_WAYPOINT].iloc[-1]
        wp = np.array(wp, dtype=np.float32)

        # --- SPEED ---
        #speed = self.samples[config.COLS_LABELS_SPEED].iloc[i]
        #speed = np.array(speed, dtype='float32')

        # --- CONTROL COMMAND ---
        #control_command = self.samples[config.COLS_LABELS_CTRL_CMD].iloc[i]
        #control_command = np.array(control_command, dtype='float32')

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(img_seq, dtype=torch.float),
            #'imu': torch.tensor(imu_seq, dtype=torch.float),
            'waypoint': torch.tensor(wp, dtype=torch.float),
            #'speed': torch.tensor(speed, dtype=torch.float),
            #'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }



class ForgetfulDataset_seq_Img2WpSpeed(ForgetfulDataset_seq):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):
        seq = self.data[i]#.reset_index(drop=True)
        seq_l = len(seq)
            
        # --- IMAGE ---
        img_seq = np.zeros((seq_l, 3, config.IMAGE_HEIGHT_RESIZED, config.IMAGE_WIDTH_RESIZED), dtype=np.float32)

        for img_i, row in seq[config.COLS_INPUT_IMAGE_FILE].iterrows():
            img_file_path = row[0]
            img = cv2.imread(img_file_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
            img = img / 255.0
            img = np.transpose(img, (2, 0, 1)) # Channel size to index 0
            img_seq[img_i] = img

        # --- IMU ---
        #imu = self.samples[config.COLS_INPUT_IMU].iloc[i]
        #imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        wp = seq[config.COLS_LABELS_WAYPOINT].iloc[-1]
        wp = np.array(wp, dtype=np.float32)

        # --- SPEED ---
        speed = seq[config.COLS_LABELS_SPEED].iloc[-1]
        speed = np.array(speed, dtype=np.float32)

        # --- CONTROL COMMAND ---
        #control_command = self.samples[config.COLS_LABELS_CTRL_CMD].iloc[i]
        #control_command = np.array(control_command, dtype='float32')

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(img_seq, dtype=torch.float),
            #'imu': torch.tensor(imu_seq, dtype=torch.float),
            'waypoint': torch.tensor(wp, dtype=torch.float),
            'speed': torch.tensor(speed, dtype=torch.float),
            #'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }



class ForgetfulDataset_seq_Img2Ctrlcmd(ForgetfulDataset_seq):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):
        seq = self.data[i]#.reset_index(drop=True)
        seq_l = len(seq)
            
        # --- IMAGE ---
        img_seq = np.zeros((seq_l, 3, config.IMAGE_HEIGHT_RESIZED, config.IMAGE_WIDTH_RESIZED), dtype=np.float32)

        for img_i, row in seq[config.COLS_INPUT_IMAGE_FILE].iterrows():
            img_file_path = row[0]
            img = cv2.imread(img_file_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
            img = img / 255.0
            img = np.transpose(img, (2, 0, 1)) # Channel size to index 0
            img_seq[img_i] = img

        # --- IMU ---
        #imu = self.samples[config.COLS_INPUT_IMU].iloc[i]
        #imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        #wp = seq[config.COLS_LABELS_WAYPOINT].iloc[-1]
        #wp = np.array(wp, dtype=np.float32)

        # --- SPEED ---
        #speed = seq[config.COLS_LABELS_SPEED].iloc[-1]
        #speed = np.array(speed, dtype=np.float32)

        # --- CONTROL COMMAND ---
        control_command = seq[config.COLS_LABELS_CTRL_CMD].iloc[-1]
        control_command = np.array(control_command, dtype=np.float32)

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(img_seq, dtype=torch.float),
            #'imu': torch.tensor(imu_seq, dtype=torch.float),
            #'waypoint': torch.tensor(wp, dtype=torch.float),
            #'speed': torch.tensor(speed, dtype=torch.float),
            'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }


class ForgetfulDataset_seq_ImgImu2Wp(ForgetfulDataset_seq):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):
        seq = self.data[i]#.reset_index(drop=True)
        seq_l = len(seq)
            
        # --- IMAGE ---
        img_seq = np.zeros((seq_l, 3, config.IMAGE_HEIGHT_RESIZED, config.IMAGE_WIDTH_RESIZED), dtype=np.float32)

        for imu_i, row in seq[config.COLS_INPUT_IMAGE_FILE].iterrows():
            img_file_path = row[0]
            img = cv2.imread(img_file_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
            img = img / 255.0
            img = np.transpose(img, (2, 0, 1)) # Channel size to index 0
            img_seq[imu_i] = img

        # --- IMU ---
        imu_seq = seq[config.COLS_INPUT_IMU]
        imu_seq = np.array(imu_seq, dtype=np.float32)
        '''imu_seq = np.zeros((seq_l, len(config.COLS_INPUT_IMU)), dtype=np.float32)

        for imu_i, row in seq[config.COLS_INPUT_IMU].iterrows():
            imu = np.array(row, dtype=np.float32)
            imu_seq[imu_i] = imu'''
        
        # --- WAYPOINT ---
        wp = seq[config.COLS_LABELS_WAYPOINT].iloc[-1]
        wp = np.array(wp, dtype=np.float32)

        # --- SPEED ---
        #speed = seq[config.COLS_LABELS_SPEED].iloc[-1]
        #speed = np.array(speed, dtype=np.float32)

        # --- CONTROL COMMAND ---
        #control_command = seq[config.COLS_LABELS_CTRL_CMD].iloc[-1]
        #control_command = np.array(control_command, dtype=np.float32)

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(img_seq, dtype=torch.float),
            'imu': torch.tensor(imu_seq, dtype=torch.float),
            'waypoint': torch.tensor(wp, dtype=torch.float),
            #'speed': torch.tensor(speed, dtype=torch.float),
            #'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }



class ForgetfulDataset_seq_ImgImu2WpSpeed(ForgetfulDataset_seq):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):
        seq = self.data[i]#.reset_index(drop=True)
        seq_l = len(seq)
            
        # --- IMAGE ---
        img_seq = np.zeros((seq_l, 3, config.IMAGE_HEIGHT_RESIZED, config.IMAGE_WIDTH_RESIZED), dtype=np.float32)

        for imu_i, row in seq[config.COLS_INPUT_IMAGE_FILE].iterrows():
            img_file_path = row[0]
            img = cv2.imread(img_file_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
            img = img / 255.0
            img = np.transpose(img, (2, 0, 1)) # Channel size to index 0
            img_seq[imu_i] = img

        # --- IMU ---
        imu_seq = seq[config.COLS_INPUT_IMU]
        imu_seq = np.array(imu_seq, dtype=np.float32)
        '''imu_seq = np.zeros((seq_l, len(config.COLS_INPUT_IMU)), dtype=np.float32)

        for imu_i, row in seq[config.COLS_INPUT_IMU].iterrows():
            imu = np.array(row, dtype=np.float32)
            imu_seq[imu_i] = imu'''
        
        # --- WAYPOINT ---
        wp = seq[config.COLS_LABELS_WAYPOINT].iloc[-1]
        wp = np.array(wp, dtype=np.float32)

        # --- SPEED ---
        speed = seq[config.COLS_LABELS_SPEED].iloc[-1]
        speed = np.array(speed, dtype=np.float32)

        # --- CONTROL COMMAND ---
        #control_command = seq[config.COLS_LABELS_CTRL_CMD].iloc[-1]
        #control_command = np.array(control_command, dtype=np.float32)

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(img_seq, dtype=torch.float),
            'imu': torch.tensor(imu_seq, dtype=torch.float),
            'waypoint': torch.tensor(wp, dtype=torch.float),
            'speed': torch.tensor(speed, dtype=torch.float),
            #'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }


class ForgetfulDataset_seq_ImgImu2Ctrlcmd(ForgetfulDataset_seq):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):
        seq = self.data[i]#.reset_index(drop=True)
        seq_l = len(seq)
            
        # --- IMAGE ---
        img_seq = np.zeros((seq_l, 3, config.IMAGE_HEIGHT_RESIZED, config.IMAGE_WIDTH_RESIZED), dtype=np.float32)

        for imu_i, row in seq[config.COLS_INPUT_IMAGE_FILE].iterrows():
            img_file_path = row[0]
            img = cv2.imread(img_file_path)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
            img = img / 255.0
            img = np.transpose(img, (2, 0, 1)) # Channel size to index 0
            img_seq[imu_i] = img

        # --- IMU ---
        imu_seq = seq[config.COLS_INPUT_IMU]
        imu_seq = np.array(imu_seq, dtype=np.float32)
        '''imu_seq = np.zeros((seq_l, len(config.COLS_INPUT_IMU)), dtype=np.float32)

        for imu_i, row in seq[config.COLS_INPUT_IMU].iterrows():
            imu = np.array(row, dtype=np.float32)
            imu_seq[imu_i] = imu'''
        
        # --- WAYPOINT ---
        #wp = seq[config.COLS_LABELS_WAYPOINT].iloc[-1]
        #wp = np.array(wp, dtype=np.float32)

        # --- SPEED ---
        #speed = seq[config.COLS_LABELS_SPEED].iloc[-1]
        #speed = np.array(speed, dtype=np.float32)

        # --- CONTROL COMMAND ---
        control_command = seq[config.COLS_LABELS_CTRL_CMD].iloc[-1]
        control_command = np.array(control_command, dtype=np.float32)

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(img_seq, dtype=torch.float),
            'imu': torch.tensor(imu_seq, dtype=torch.float),
            #'waypoint': torch.tensor(wp, dtype=torch.float),
            #'speed': torch.tensor(speed, dtype=torch.float),
            'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }


class ForgetfulDataset(Dataset):
    def __init__(self, samples, name):
        self.data = samples
        self.name = name
    def __len__(self):
        return len(self.data)
    def __getitem__(self, i):
        pass
    def plot_samples(self):
        
        plt.figure(figsize=(10, 10))
        
        for sample_i in range(9):

            sample = self.__getitem__(sample_i)

            img = sample['image']
            #img = np.array(img, dtype='float32')
            img = img.cpu().detach().numpy()
            img = np.transpose(img, (1, 2, 0))
            
            labels = sample['labels']
            wp_x = labels[0].item()
            wp_y = labels[1].item()
            v = labels[2].item()

            wp_x = int(config.IMAGE_WIDTH_RESIZED / 2 * (1 + wp_x))
            wp_y = int(config.IMAGE_HEIGHT_RESIZED / 2 * (1 + wp_y))

            img = cv2.line(img, (wp_x, config.IMAGE_HEIGHT_RESIZED-1), (wp_x, 0), color=(0, 1, 0), thickness=1)
            img = cv2.line(img, (0, wp_y), (config.IMAGE_WIDTH_RESIZED-1, wp_y), color=(0, 1, 0), thickness=1)

            plt.subplot(3, 3, sample_i+1)
            plt.imshow(img)
            #plt.scatter(wp_x, wp_y, c ="green")
            plt.text(config.IMAGE_WIDTH_RESIZED*0.1, 
                config.IMAGE_HEIGHT_RESIZED*0.1, f"v={v:.2f}", c="green", weight="bold")
        
        plt.title(f"{self.name}")
        plt.savefig(f"{config.OUTPUT_DIR}/_data_vis_check/{self.name}.png")
        if config.RUN_SHOW_PLOT:
            plt.show()
        plt.close()




class ForgetfulDataset_Img2Wp(ForgetfulDataset):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):

        # --- IMAGE ---
        image = cv2.imread(self.data[config.COLS_INPUT_IMAGE_FILE].iloc[i][0])
        image = cv2.resize(image, 
            (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image / 255.0
        image = np.transpose(image, (2, 0, 1)) # Channel size to index 0

        # --- IMU ---
        #imu = self.samples[config.COLS_INPUT_IMU].iloc[i]
        #imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        waypoint = self.data[config.COLS_LABELS_WAYPOINT].iloc[i]
        waypoint = np.array(waypoint, dtype='float32')

        # --- SPEED ---
        #speed = self.samples[config.COLS_LABELS_SPEED].iloc[i]
        #speed = np.array(speed, dtype='float32')

        # --- CONTROL COMMAND ---
        #control_command = self.samples[config.COLS_LABELS_CTRL_CMD].iloc[i]
        #control_command = np.array(control_command, dtype='float32')

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(image, dtype=torch.float),
            #'imu': torch.tensor(imu, dtype=torch.float),
            'waypoint': torch.tensor(waypoint, dtype=torch.float),
            #'speed': torch.tensor(speed, dtype=torch.float),
            #'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }



class ForgetfulDataset_Img2Wp_Seq(ForgetfulDataset):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):

        # --- IMAGE ---
        image = cv2.imread(self.data[config.COLS_INPUT_IMAGE_FILE].iloc[i][0])
        image = cv2.resize(image, 
            (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image / 255.0
        image = np.transpose(image, (2, 0, 1)) # Channel size to index 0

        # --- IMU ---
        #imu = self.samples[config.COLS_INPUT_IMU].iloc[i]
        #imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        waypoint = self.data[config.COLS_LABELS_WAYPOINT].iloc[i]
        waypoint = np.array(waypoint, dtype='float32')

        # --- SPEED ---
        #speed = self.samples[config.COLS_LABELS_SPEED].iloc[i]
        #speed = np.array(speed, dtype='float32')

        # --- CONTROL COMMAND ---
        #control_command = self.samples[config.COLS_LABELS_CTRL_CMD].iloc[i]
        #control_command = np.array(control_command, dtype='float32')

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(image, dtype=torch.float),
            #'imu': torch.tensor(imu, dtype=torch.float),
            'waypoint': torch.tensor(waypoint, dtype=torch.float),
            #'speed': torch.tensor(speed, dtype=torch.float),
            #'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }



class ForgetfulDataset_Img2WpSpeed(ForgetfulDataset):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):

        # --- IMAGE ---
        image = cv2.imread(self.data[config.COLS_INPUT_IMAGE_FILE].iloc[i][0])
        image = cv2.resize(image, 
            (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image / 255.0
        image = np.transpose(image, (2, 0, 1)) # Channel size to index 0

        # --- IMU ---
        #imu = self.samples[config.COLS_INPUT_IMU].iloc[i]
        #imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        waypoint = self.data[config.COLS_LABELS_WAYPOINT].iloc[i]
        waypoint = np.array(waypoint, dtype='float32')

        # --- SPEED ---
        speed = self.data[config.COLS_LABELS_SPEED].iloc[i]
        speed = np.array(speed, dtype='float32')

        # --- CONTROL COMMAND ---
        #control_command = self.samples[config.COLS_LABELS_CTRL_CMD].iloc[i]
        #control_command = np.array(control_command, dtype='float32')

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(image, dtype=torch.float),
            #'imu': torch.tensor(imu, dtype=torch.float),
            'waypoint': torch.tensor(waypoint, dtype=torch.float),
            'speed': torch.tensor(speed, dtype=torch.float),
            #'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }


class ForgetfulDataset_ImgImu2Wp(ForgetfulDataset):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):

        # --- IMAGE ---
        image = cv2.imread(self.data[config.COLS_INPUT_IMAGE_FILE].iloc[i][0])
        image = cv2.resize(image, 
            (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image / 255.0
        image = np.transpose(image, (2, 0, 1)) # Channel size to index 0

        # --- IMU ---
        imu = self.data[config.COLS_INPUT_IMU].iloc[i]
        imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        waypoint = self.data[config.COLS_LABELS_WAYPOINT].iloc[i]
        waypoint = np.array(waypoint, dtype='float32')

        # --- SPEED ---
        #speed = self.samples[config.COLS_LABELS_SPEED].iloc[i]
        #speed = np.array(speed, dtype='float32')

        # --- CONTROL COMMAND ---
        #control_command = self.samples[config.COLS_LABELS_CTRL_CMD].iloc[i]
        #control_command = np.array(control_command, dtype='float32')

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(image, dtype=torch.float),
            'imu': torch.tensor(imu, dtype=torch.float),
            'waypoint': torch.tensor(waypoint, dtype=torch.float),
            #'speed': torch.tensor(speed, dtype=torch.float),
            #'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }



class ForgetfulDataset_ImgImu2WpSpeed(ForgetfulDataset):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):

        # --- IMAGE ---
        image = cv2.imread(self.data[config.COLS_INPUT_IMAGE_FILE].iloc[i][0])
        image = cv2.resize(image, 
            (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image / 255.0
        image = np.transpose(image, (2, 0, 1)) # Channel size to index 0

        # --- IMU ---
        imu = self.data[config.COLS_INPUT_IMU].iloc[i]
        imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        waypoint = self.data[config.COLS_LABELS_WAYPOINT].iloc[i]
        waypoint = np.array(waypoint, dtype='float32')

        # --- SPEED ---
        speed = self.data[config.COLS_LABELS_SPEED].iloc[i]
        speed = np.array(speed, dtype='float32')

        # --- CONTROL COMMAND ---
        #control_command = self.samples[config.COLS_LABELS_CTRL_CMD].iloc[i]
        #control_command = np.array(control_command, dtype='float32')

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(image, dtype=torch.float),
            'imu': torch.tensor(imu, dtype=torch.float),
            'waypoint': torch.tensor(waypoint, dtype=torch.float),
            'speed': torch.tensor(speed, dtype=torch.float),
            #'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }




class ForgetfulDataset_Img2Ctrlcmd(ForgetfulDataset):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):

        # --- IMAGE ---
        image = cv2.imread(self.data[config.COLS_INPUT_IMAGE_FILE].iloc[i][0])
        image = cv2.resize(image, 
            (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image / 255.0
        image = np.transpose(image, (2, 0, 1)) # Channel size to index 0

        # --- IMU ---
        #imu = self.samples[config.COLS_INPUT_IMU].iloc[i]
        #imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        #waypoint = self.samples[config.COLS_LABELS_WAYPOINT].iloc[i]
        #waypoint = np.array(waypoint, dtype='float32')

        # --- SPEED ---
        #speed = self.samples[config.COLS_LABELS_SPEED].iloc[i]
        #speed = np.array(speed, dtype='float32')

        # --- CONTROL COMMAND ---
        control_command = self.data[config.COLS_LABELS_CTRL_CMD].iloc[i]
        control_command = np.array(control_command, dtype='float32')

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(image, dtype=torch.float),
            #'imu': torch.tensor(imu, dtype=torch.float),
            #'waypoint': torch.tensor(waypoint, dtype=torch.float),
            #'speed': torch.tensor(speed, dtype=torch.float),
            'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }




class ForgetfulDataset_ImgImu2Ctrlcmd(ForgetfulDataset):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):

        # --- IMAGE ---
        image = cv2.imread(self.data[config.COLS_INPUT_IMAGE_FILE].iloc[i][0])
        image = cv2.resize(image, 
            (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image / 255.0
        image = np.transpose(image, (2, 0, 1)) # Channel size to index 0

        # --- IMU ---
        imu = self.data[config.COLS_INPUT_IMU].iloc[i]
        imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        #waypoint = self.samples[config.COLS_LABELS_WAYPOINT].iloc[i]
        #waypoint = np.array(waypoint, dtype='float32')

        # --- SPEED ---
        #speed = self.samples[config.COLS_LABELS_SPEED].iloc[i]
        #speed = np.array(speed, dtype='float32')

        # --- CONTROL COMMAND ---
        control_command = self.data[config.COLS_LABELS_CTRL_CMD].iloc[i]
        control_command = np.array(control_command, dtype='float32')

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(image, dtype=torch.float),
            'imu': torch.tensor(imu, dtype=torch.float),
            #'waypoint': torch.tensor(waypoint, dtype=torch.float),
            #'speed': torch.tensor(speed, dtype=torch.float),
            'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }




class ForgetfulDataset_Img_IMU_Wp_Speed_CtrlCmd(ForgetfulDataset):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, i):

        # --- IMAGE ---
        image = cv2.imread(self.data[config.COLS_INPUT_IMAGE_FILE].iloc[i][0])
        image = cv2.resize(image, 
            (config.IMAGE_WIDTH_RESIZED, config.IMAGE_HEIGHT_RESIZED))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image / 255.0
        image = np.transpose(image, (2, 0, 1)) # Channel size to index 0

        # --- IMU ---
        imu = self.data[config.COLS_INPUT_IMU].iloc[i]
        imu = np.array(imu, dtype='float32')

        # --- WAYPOINT ---
        waypoint = self.data[config.COLS_LABELS_WAYPOINT].iloc[i]
        waypoint = np.array(waypoint, dtype='float32')

        # --- SPEED ---
        speed = self.data[config.COLS_LABELS_SPEED].iloc[i]
        speed = np.array(speed, dtype='float32')

        # --- CONTROL COMMAND ---
        control_command = self.data[config.COLS_LABELS_CTRL_CMD].iloc[i]
        control_command = np.array(control_command, dtype='float32')

        # --- RETURN DICT ---
        return {
            'image': torch.tensor(image, dtype=torch.float),
            'imu': torch.tensor(imu, dtype=torch.float),
            'waypoint': torch.tensor(waypoint, dtype=torch.float),
            'speed': torch.tensor(speed, dtype=torch.float),
            'ctrl_cmd': torch.tensor(control_command, dtype=torch.float),
            }"""














'''def get_dataloader(FILE, train=True, sequential=False):

    dataset_name = FILE.name
    data = pd.read_csv(FILE, usecols=config.COLS_INPUTS_AND_LABELS)

    # Take only specified number of subsequent images at random position
    if not config.RUN_IMAGE_N == 'ALL':
        sample_n = len(data)
        rand_idx = random.randint(0, sample_n - config.RUN_IMAGE_N)
        data = data[rand_idx : rand_idx + config.RUN_IMAGE_N]
        data = data.reset_index(drop=True)
        sample_n = len(data)

    if not sequential:
        if config.IMG__2__WP:
            dataset = ForgetfulDataset_Img2Wp(data, dataset_name)
        elif config.IMG__2__WP_SPEED:
            dataset = ForgetfulDataset_Img2WpSpeed(data, dataset_name)
        elif config.IMG__2__CTRLCMD:
            dataset = ForgetfulDataset_Img2Ctrlcmd(data, dataset_name)
        elif config.IMG_IMU__2__WP:
            ForgetfulDataset_ImgImu2Wp(data, dataset_name)
        elif config.IMG_IMU__2__WP_SPEED:
            dataset = ForgetfulDataset_ImgImu2WpSpeed(data, dataset_name)
        elif config.IMG_IMU__2__CTRLCMD:
            dataset = ForgetfulDataset_ImgImu2Ctrlcmd(data, dataset_name)

        dataloader = DataLoader(
            dataset,
            batch_size=config.BATCH_SIZE, 
            shuffle=True if train else False,
            drop_last=True
        )

        return dataloader, dataset

    else: #(if sequential)
        sample_n = len(data)
        data_seq = list()

        if train:
            seq_l = config.RNN_SEQUENCE_LEN
            #step = max(int(seq_l * config.RNN_SEQUENCE_REL_OVERLAP), 1)
            step = config.RNN_SEQUENCE_STEP
            last_possible_idx = sample_n - seq_l
            rand_start_idx = random.randrange(1 + last_possible_idx%step)
            for i in range(rand_start_idx, last_possible_idx, step):
                data_seq.append(data[i:i + seq_l].reset_index(drop=True))
        else: #if valid
            seq_l = 1
            step = 1
            for i in range(0, sample_n - seq_l, step):
                data_seq.append(data[i:i + seq_l].reset_index(drop=True))

        if config.IMG__2__WP:
            dataset = ForgetfulDataset_seq_Img2Wp(data_seq, dataset_name)
        elif config.IMG__2__WP_SPEED:
            #raise AssertionError(f"'ForgetfulDataset_seq_Img2WpSpeed' not implemented yet.")
            dataset = ForgetfulDataset_seq_Img2WpSpeed(data_seq, dataset_name)
        elif config.IMG__2__CTRLCMD:
            #raise AssertionError(f"'ForgetfulDataset_seq_Img2Ctrlcmd' not implemented yet.")
            dataset = ForgetfulDataset_seq_Img2Ctrlcmd(data_seq, dataset_name)
        elif config.IMG_IMU__2__WP:
            #raise AssertionError(f"'ForgetfulDataset_seq_ImgImu2Wp' not implemented yet.")
            dataset = ForgetfulDataset_seq_ImgImu2Wp(data_seq, dataset_name)
        elif config.IMG_IMU__2__WP_SPEED:
            #raise AssertionError(f"'ForgetfulDataset_seq_ImgImu2WpSpeed' not implemented yet.")
            dataset = ForgetfulDataset_seq_ImgImu2WpSpeed(data_seq, dataset_name)
        elif config.IMG_IMU__2__CTRLCMD:
            #raise AssertionError(f"'ForgetfulDataset_seq_ImgImu2Ctrlcmd' not implemented yet.")
            dataset = ForgetfulDataset_seq_ImgImu2Ctrlcmd(data_seq, dataset_name)
        
        dataloader = DataLoader(
            dataset,
            batch_size=config.BATCH_SIZE if train else 1, 
            shuffle=True if train else False,
            drop_last=True
            )

        return dataloader, dataset







def get_dataloaders(sequential=False):

    run_train_files, run_valid_files = get_run_files()
    run_train_n = len(run_train_files)
    run_valid_n = len(run_valid_files)
    
    print("")
    print("")
    print(".--- GENERATE DATA LOADERS")
    print("|")
    print(f"|  - Mode: {config.RUN_SELECTION_MODE}")
    print(f"|  - Number of runs")
    print(f"|     - Training: {run_train_n}")# + " (ALL_REMAINING)" if config.TRAIN_RUN_N_PER_SCENE == 'ALL_REMAINING' else "")
    print(f"|     - Validation: {run_valid_n}")
    print(f"|  - Number of images per run: {config.RUN_IMAGE_N}")
    print(f"|  - Sequence length/step: {config.RNN_SEQUENCE_LEN}/{config.RNN_SEQUENCE_STEP}")
    print(f"|  - Inputs: {config.COLS_INPUTS}")
    print(f"|  - Labels: {config.COLS_LABELS}")
    print("|")

    dataloaders = dict()
    dataloaders["train"] = dict()
    dataloaders["valid"] = dict()
    datasets = list()


    with tqdm(enumerate(run_train_files)) as TQDM:
        for i, FILE in TQDM:
            run_name = FILE.name

            TQDM.set_description(f"|  Training run data loaders")
            TQDM.set_postfix({'run': run_name})

            if i%4==0: TQDM.write(f"|    {run_name}", end='')
            elif i%4==3: TQDM.write(f"    {run_name}", end='\n')
            else: TQDM.write(f"    {run_name}", end='')
            
            if i+1==run_train_n and i%4!=3: 
                TQDM.write("", end='\n')

            
            dataloader, dataset = get_dataloader(FILE, train=True, sequential=sequential)
            datasets.append(dataset)
            dataloaders["train"][run_name] = dataloader
    
    if sequential:
        dataset = datasets[0]
        datas = [x.data for x in datasets]
        dataset.data = [sequence for data in datas for sequence in data]
        dataloader = DataLoader(
            dataset,
            batch_size=config.BATCH_SIZE, 
            shuffle=True,
            drop_last=True
            )
        dataloaders['train'] = dict()
        dataloaders['train']['runs_combined'] = dataloader
    
        
        

    print('|')
    with tqdm(enumerate(run_valid_files)) as TQDM:
        for i, FILE in TQDM:
            run_name = FILE.name

            TQDM.set_description(f"|  Validation run data loaders")
            TQDM.set_postfix({'run': run_name})

            if i%4==0: TQDM.write(f"|    {run_name}", end='')
            elif i%4==3: TQDM.write(f"    {run_name}", end='\n')
            else: TQDM.write(f"    {run_name}", end='')

            if i+1==run_valid_n and i%4!=3: 
                TQDM.write("", end='\n')

            dataloader, _ = get_dataloader(FILE, train=False, sequential=sequential)
            dataloaders["valid"][run_name] = dataloader

    print("|_________________________________________________")

    return dataloaders'''










"""RUN_FILES_AVAILABLE = sorted([
    x for x in config.INPUT_DIR.iterdir()\
        if x.is_file()\
            and x.name[:4] == "run_"\
            and x.name[-4:] == ".csv"
])

if not run_files_exist(): generate_run_files()
elif not run_files_paths_exist(): generate_run_files()


dataloaders = get_dataloaders(sequential=config.USE_RNN)"""

