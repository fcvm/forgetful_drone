import matplotlib.pyplot as plt
import json
import rospkg; rospack = rospkg.RosPack()
from pathlib import Path


#fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

#LS = [':']*6 + ['-']*6
cnt=0
TL = []
VL = []
LR = []
for EXP in [
    ['resnet18_1third_trainable', 'b:'],
    ['resnet18_1third_frozenbutlast', 'g:'],
    ['resnet18_1third_first3layers', 'r:'],
    ['resnet18_1third_first2layers', 'c:'],
    ['resnet8_1third', 'm:'],
    ['resnet8_1third_noreg', 'y:'],
    ['resnet18_1half_trainable', 'b-'],
    ['resnet18_1half_frozenbutlast', 'g-'],
    ['resnet18_1half_first3layers', 'r-'],
    ['resnet18_1half_first2layers', 'c-'],
    ['resnet8_1half', 'm-'],
    ['resnet8_1half_noreg', 'y-'],
]:
    

    p = Path (rospack.get_path ('forgetful_drones'))/f'experiments/{EXP[0]}/output/train_records.json'
    with open (p, 'r') as f: rec = json.load (f)
    TL.append ([x ['train_loss'] for x in rec])
    VL.append ([x ['valid_loss'] for x in rec])
    LR.append ([x ['learn_rate'] for x in rec])

    ax1.plot ([x ['train_loss'] for x in rec][:100], EXP[1], label=EXP[0])#, linestyle=':' if EXP.split('_')[1]=='1third' else '-')
    ax2.plot ([x ['valid_loss'] for x in rec][:100], EXP[1], label=EXP[0])#, linestyle=':' if EXP.split('_')[1]=='1third' else '-')
    #ax3.plot ([x ['learn_rate'] for x in rec][:100], label=EXP, linestyle=':' if EXP.split('_')[1]=='1third' else '-')

    cnt+=1



#ax3.set_xlabel ('Epochs')
ax2.set_xlabel ('Epochs')

ax1.set_ylabel ('Train Loss')
ax2.set_ylabel ('Valid Loss')
#ax3.set_ylabel ('Learn rate')

ax1.legend ()
ax2.legend ()
#ax3.legend ()     

ax1.set_yscale ('log')
ax2.set_yscale ('log')
#ax3.set_yscale ('log')

ax1.grid ()
ax2.grid ()
#ax3.grid ()

plt.show ()


    


    