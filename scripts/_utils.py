from pathlib import Path
import rospkg; rospack = rospkg.RosPack ()


def find_parent_with_element (p: Path, n: str) -> Path:
    """
    Find the first, p-upward directory containing an element named n
    """
    if not n in [x.name for x in p.iterdir ()]:
        p = find_parent_with_element (p.parent.resolve (), n)
    return p

















def clear_raw_run_dirs (exp_id : str) -> None:
    _raw_ = Path (rospack.get_path ('forgetful_drones'))/'experiments'/exp_id/'data/raw'
    
    print (f"Clear raw run dirs of experiment: {exp_id}")
    run_cnt = 0
    for _run_ in _raw_.iterdir ():
        _rgb_ = _run_/'rgb'
        if _rgb_.is_dir ():
            for _RGB in _rgb_.iterdir ():
                _RGB.unlink ()
            _rgb_.rmdir ()

        _DAT = _run_/'data.txt'
        _DAT.unlink (missing_ok=True)
        
        run_cnt += 1
        print (f"  - Cleared contents of \"{_run_}\"")
    
    print (f"Cleared contents of {run_cnt} runs\n")