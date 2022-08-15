from ast import literal_eval
import torch
from typing import List
import cv2
import numpy as np
import pandas as pd


class ForgetfulDataset_old (torch.utils.data.Dataset):
    
    def __init__ (
        self, 
        df: pd.DataFrame, 
        id: str, 
        cnn_c: int, 
        cnn_h: int, 
        cnn_w: int, 
        cnn_col: str,
        cat_cols: List[str],
        lbl_cols: List[str],
        *args, **kwargs
    ) -> None:

        
        super().__init__(*args, **kwargs)
        self.df = df
        self.seq_len = len(df.iloc[0][0])
        self.id = id

        self.cnn_c = cnn_c#3
        self.cnn_h = cnn_h#200
        self.cnn_w = cnn_w#300

        self.cnn_col = cnn_col
        self.cat_cols = cat_cols
        self.lbl_cols = lbl_cols
        
       
    def __len__ (self):
        return len(self.df)
    
    def __getitem__(self, i: int):
        sequence = self.df.iloc[i]
        
        cnn_seq = np.zeros((self.seq_len, self.cnn_c, self.cnn_h, self.cnn_w), dtype=np.float32)
        for i, rgb_fpath in enumerate(sequence[self.cnn_col]):
            rgb = cv2.imread(rgb_fpath)
            rgb = cv2.resize(rgb, (self.cnn_w, self.cnn_h))
            rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
            rgb = rgb / 255.0
            rgb = np.transpose(rgb, (2, 0, 1)) # channel size to index 0
            cnn_seq[i] = rgb
        
        cat_seq = np.array(list(sequence[self.cat_cols]), dtype=np.float32).T
        label = np.array(sequence[self.lbl_cols], dtype=np.float32)

        return {
            'input': {
                'cnn': torch.tensor(cnn_seq, dtype=torch.float),
                'cat': torch.tensor(cat_seq, dtype=torch.float)
            },
            'label': torch.tensor(label, dtype=torch.float)
        }



class ForgetfulDataset (torch.utils.data.Dataset):
    
    def __init__ (
        self, 
        df: pd.DataFrame, 
        id: str, 
        cnn_cols: str,
        cat_cols: List[str],
        lbl_cols: List[str],
        msg_pfx:str='',
        *args, **kwargs
    ) -> None:
    
        super().__init__(*args, **kwargs)

        self.df = df
        #self.df['max_speed'] = self.df['max_speed'].apply(lambda x: literal_eval(x) if isinstance(x, str) else x)
        self.id = id

        self.cnn_cols = cnn_cols
        self.cat_cols = cat_cols
        self.lbl_cols = lbl_cols

        sequential_length = len(df.iloc[0][0])
        fpath = next(iter(self.df.iloc[0][self.cnn_cols[0]]))
        cnn_height, cnn_width, cnn_num_channels = cv2.imread(fpath).shape
        self.cnn_seq = np.zeros(
            (
                sequential_length, 
                cnn_num_channels, 
                cnn_height,
                cnn_width
            ), 
            dtype=np.float32
        )

        self.addToConsoleRepresentation (f"")
        self.addToConsoleRepresentation (f".--- INIT FORGETFUL DATASET")
        self.addToConsoleRepresentation (f"|")
        self.addToConsoleRepresentation (f"|  EXP  |    - ID: {self.id}")
        self.addToConsoleRepresentation (f"|  SEQ  |    - Num: {len(self.df)}  - Len: {sequential_length}")
        self.addToConsoleRepresentation (f"|  CNN  |    - Cols: {self.cnn_cols}  - shape: {cnn_num_channels}x{cnn_height}x{cnn_width}")
        self.addToConsoleRepresentation (f"|  CAT  |    - Cols: {self.cat_cols}")
        self.addToConsoleRepresentation (f"|  LBL  |    - Cols: {self.lbl_cols}")
        self.addToConsoleRepresentation (f"|_________________________________________________")
        print (self.console_representation)

        

        
    def __len__ (self):
        return len(self.df)
    
    def __getitem__(self, i: int):
        sequence = self.df.iloc[i]

        for i, rgb_fpath in enumerate(sequence[self.cnn_cols[0]]):
            rgb = cv2.imread(rgb_fpath)
            rgb = np.array(rgb / 255.0, dtype=np.float32)
            rgb = np.transpose(rgb, (2, 0, 1)) # channel size to index 0
            self.cnn_seq[i] = rgb
        cat_seq = np.array(list(sequence[self.cat_cols]), dtype=np.float32).T
        label = np.array(sequence[self.lbl_cols], dtype=np.float32)

        return {
            'input': {
                'cnn': torch.tensor(self.cnn_seq, dtype=torch.float),
                'cat': torch.tensor(cat_seq, dtype=torch.float)
            },
            'label': torch.tensor(label, dtype=torch.float)
        }

    
    def addToConsoleRepresentation (self, text: str) -> None:
        try: self.console_representation += text + '\n'
        except: self.console_representation = text + '\n'