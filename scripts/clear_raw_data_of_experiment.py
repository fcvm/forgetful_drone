#!/usr/bin/env python3


EXP_ID : str = 'UTC_2022_5_26_13_53_25'
import pathlib

def find_root_dpath (path: pathlib.Path) -> pathlib.Path:
    print(path)
    if not 'experiments' in [x.name for x in path.iterdir()]:
        path = find_root_dpath (path.parent.resolve())
    return path


ROOT_DPATH = find_root_dpath(pathlib.Path(__file__).parent.resolve())
raw_dpath = ROOT_DPATH/'experiments'/EXP_ID/'data/raw'
for run_dpath in raw_dpath.iterdir():
    try:
        rgb_dpath = run_dpath/'rgb'
        for rgb_fpath in rgb_dpath.iterdir():
            rgb_fpath.unlink();
        rgb_dpath.rmdir()
    except: pass

    try:
        data_fpath = run_dpath/'data.txt'
        data_fpath.unlink()
    except: pass