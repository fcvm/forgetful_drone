from forgetful_brain import ForgetfulBrain; fb = ForgetfulBrain ()

for EXP_ID in [
    #'UTC_2022_06_13_10_13_56',
    #'UTC_2022_06_22_21_15_38',
    #'UTC_2022_07_01_07_27_56',
    #'UTC_2022_07_01_07_27_56___SEQLEN_2',
    #'UTC_2022_07_01_07_27_56___SEQLEN_5',
    #'UTC_2022_07_01_07_27_56___SEQLEN_10',
    #'UTC_2022_07_01_07_27_56___SEQLEN_25',
    #'UTC_2022_07_01_07_27_56___SEQLEN_2___RF3',
    #'UTC_2022_07_01_07_27_56___SEQLEN_3___RF3',
    #'UTC_2022_07_01_07_27_56___SEQLEN_5___RF3', 
    #'UTC_2022_07_01_07_27_56___SEQLEN_10___RF3',
    #
    #'UTC_2022_06_13_10_13_56_+'
    #'resnet18_1third_first2layers'
    'UTC_2022_09_14_07_18_53___E1R1_3GRUlayer'
]:

    fb.initExp_resetTraining (EXP_ID)
    fb.startTrain (100, reset_lrsched=True)
