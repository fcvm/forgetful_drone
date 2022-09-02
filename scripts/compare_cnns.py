import matplotlib.pyplot as plt
import json
import rospkg; rospack = rospkg.RosPack()
from pathlib import Path


fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

TL = []
VL = []
LR = []
for EXP in [
    'resnet18_1third_trainable',
    #'resnet18_1third_frozenbutlast',
    #'resnet18_1third_first3layers',
    #'resnet18_1third_first2layers',
    #'resnet8_1third',
    #'resnet8_1third_noreg',
]:

    p = Path (rospack.get_path ('forgetful_drones'))/f'experiments/{EXP}/output/train_records.json'
    with open (p, 'r') as f: rec = json.load (f)
    TL.append ([x ['train_loss'] for x in rec])
    VL.append ([x ['valid_loss'] for x in rec])
    LR.append ([x ['learn_rate'] for x in rec])

    ax1.plot ([x ['train_loss'] for x in rec][:100], label=EXP)
    ax2.plot ([x ['valid_loss'] for x in rec][:100], label=EXP)
    ax3.plot ([x ['learn_rate'] for x in rec][:100], label=EXP)



ax3.set_xlabel ('Epochs')

ax1.set_ylabel ('Train Loss')
ax2.set_ylabel ('Valid Loss')
ax3.set_ylabel ('Learn rate')

ax1.legend ()
ax2.legend ()
ax3.legend ()     

ax1.set_yscale ('log')
ax2.set_yscale ('log')
ax3.set_yscale ('log')

plt.show ()


    


    