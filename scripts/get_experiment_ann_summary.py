from forgetful_brain import ForgetfulBrain; fb = ForgetfulBrain ()

for EXP_ID in [
    #   'UTC_2022_06_13_10_13_56',
    #'UTC_2022_06_22_21_15_38',
    #'UTC_2022_07_01_07_27_56',
    #'UTC_2022_07_01_07_27_56___SEQLEN_2',
    #'UTC_2022_07_01_07_27_56___SEQLEN_5',
    #'UTC_2022_07_01_07_27_56___SEQLEN_10',
    #'UTC_2022_07_01_07_27_56___SEQLEN_25',
    #'UTC_2022_07_01_07_27_56___SEQLEN_2___RF3',
    'UTC_2022_07_01_07_27_56___SEQLEN_3___RF3',
    #'UTC_2022_07_01_07_27_56___SEQLEN_5___RF3', 
    #'UTC_2022_07_01_07_27_56___SEQLEN_10___RF3',
]:

    fb.initExp (EXP_ID)
    fb.getModelSummary ()
