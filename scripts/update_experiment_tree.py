

import json
import rospkg; rospack = rospkg.RosPack()
from pathlib import Path
import filecmp



#EXP_ID = '__/incomplete/UTC_2022_06_02_18_57_40'
#EXP_ID = 'UTC_2022_06_13_10_13_56'
EXP_ID = 'UTC_2022_07_01_07_27_56___SEQLEN_2'

################################################################
################################################################
################################################################
################################################################
################################################################

def rmtree (p: Path) -> None:
    for s in p.iterdir (): s.unlink () if s.is_file () else rmtree (s)
    p.rmdir ()
setattr (Path, 'rmtree', rmtree)

def old_to_new (o : Path, n: Path, s: str) -> None:
    if o.is_file ():
        if n.is_file ():
            raise FileExistsError (f"[{s}]  Found old and new file")
        else:
            print (f"[{s}]  Move old to new file")
            n.write_bytes (o.read_bytes ())
            if filecmp.cmp(n, o): 
                o.unlink ()
            else:
                raise AssertionError (f"[{s}]  Old and new file are different")
    else:
        if n.is_file ():
            print (f"[{s}]  Already moved old to new file")
        else:
            raise FileExistsError (f"[{s}]  Found neither old nor new file")





EXP = Path (rospack.get_path ('forgetful_drones'))/'experiments'/EXP_ID
CNF = EXP/'config'
DAT = EXP/'data'
PRC = DAT/'processed'
IMD = DAT/'intermediate'
RAW = DAT/'raw'
OUT = EXP/'output'
PLT = OUT/'plot'; PLT.mkdir (exist_ok=True)
TMP = OUT/'tmp'; TMP.mkdir (exist_ok=True)







name = 'config.json'
old = OUT/name
new = CNF/name
old_to_new (old, new, name)

name = 'expert_intervention_share.pdf'
old = OUT/name
new = PLT/name
old_to_new (old, new, name)

name = 'recordings_log_y.pdf -> loss.pdf'
old = OUT/'recordings_log_y.pdf'
new = PLT/'loss.pdf'
old_to_new (old, new, name)

old = OUT/'recordings_lin_y.pdf'; old.unlink (missing_ok=True)
old = OUT/'user_input.json'; old.unlink (missing_ok=True)


## recordings.json -> train_records.json

name = 'recordings.json -> train_records.json'
old = OUT/'recordings.json'
new = OUT/'train_records.json'


if new.is_file (): print (f"[{name}]  Already done")
else:
    if not old.is_file ():
        raise FileExistsError (f"[{name}]  Found neither old nor new file")
    else:
        print (f"[{name}]  Process")
        with open (old, 'r') as f: recordings = json.load (f)
        lr_list = [el for lst in recordings ['learn_rate'] for el in lst]
        tl_list = [el for lst in recordings ['loss'] ['train'] for el in lst]
        vl_list = [el for lst in recordings ['loss'] ['valid'] for el in lst]
        train_records = []
        for i in range (len (lr_list)):
            train_records.append ({
                'learn_rate': lr_list [i],
                'train_loss': tl_list [i],
                'valid_loss': vl_list [i],
            })
        with open (new, 'w') as f: f.write (json.dumps (train_records, sort_keys=True, indent=4))


name = 'build_records.json'
new = OUT/'build_records.json'

if new.is_file (): print (f"[{name}]  Already done")
else:
    print (f"[{name}]  Process")
    input (f"Create symlink to raw data (ln -s <target> {RAW}), then press Enter to continue...")

    if IMD.is_dir (): IMD.rmtree ()
    if PRC.is_dir (): PRC.rmtree ()

    IMD.mkdir ()
    PRC.mkdir ()
    
    (PRC/'processed.csv').touch ()
    with open (OUT/'build_records.json', 'w') as f: 
        f.write (json.dumps ([], sort_keys=True, indent=4))

    from forgetful_brain import ForgetfulBrain
    fb = ForgetfulBrain ()

    fb.initExp (EXP_ID)
    fb.rebuildRuns ()
