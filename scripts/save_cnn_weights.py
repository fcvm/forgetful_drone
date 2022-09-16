from forgetful_brain import ForgetfulBrain; fb = ForgetfulBrain ()

for EXP_ID in [
    #'resnet8_1half',
    #'resnet8_1half_noreg',
    #'resnet8_1third',
    #'resnet8_1third_noreg',
    #'resnet18_1half_first2layers',
    'resnet18_1half_first3layers',
    #'resnet18_1half_frozenbutlast',
    #'resnet18_1half_trainable',
    #'resnet18_1third_first2layers',
    'resnet18_1third_first3layers',
    #'resnet18_1third_frozenbutlast',
    #'resnet18_1third_trainable',
]:

    fb.initExp (EXP_ID)
    fb.saveCNNWeights ()
