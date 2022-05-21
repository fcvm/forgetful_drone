import pathlib
import os
import subprocess

DATASET_NAME = "PARIS"


TRAINING_DATA_DIR_PATH = pathlib.Path(__file__).parent.resolve()
ROOT = str(TRAINING_DATA_DIR_PATH) + "/" + DATASET_NAME


#RECURSIVE_SUBDIR_PATHS = [x[0] for x in os.walk(TRAINING_DATA_DIR_PATH)]
#
#print(RECURSIVE_SUBDIR_PATHS)

for RUN_IMAGES_DIR_PATH in sorted(pathlib.Path(ROOT).rglob('images/')):
    
    #print(RUN_IMAGES_DIR_PATH)

    cmd = "cd " + str(RUN_IMAGES_DIR_PATH) + ";"
    cmd += "ls -1v *.jpg > files.txt;"
    cmd += "mencoder -nosound -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=21600000 -o video.avi -mf type=jpeg:fps=50 mf://@files.txt -vf scale=720:480;"
    #cmd += "ffmpeg -i video.avi " + str(RUN_IMAGES_DIR_PATH.parents[0]) + "/" + str(RUN_IMAGES_DIR_PATH.parents[0].name) + ".mp4;"
    cmd += "N | ffmpeg -i video.avi " + ROOT + "/videos/" + str(RUN_IMAGES_DIR_PATH.parents[0].name) + ".mp4;"
    cmd += "rm video.avi; rm files.txt;"
    #print(cmd)
    
    returned_value = subprocess.call(cmd, shell=True)  # returns the exit code in unix
    #print('returned value:', returned_value)