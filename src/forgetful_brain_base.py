

#!/usr/bin/env python


import os
import datetime
from pprint import PrettyPrinter
import sys
import gflags
from keras.utils.data_utils import get_file
import keras_preprocessing
from tensorboard.compat import tf2
from tensorflow._api.v2 import data

import rospy
import tensorflow as tf


# C&p-ed imports
#from ddr_learner.common_flags import FLAGS
#from Network import Network
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped

#from ddr_learner.models.base_learner import TrajectoryLearner
import time
import math
from itertools import count
import random

from keras.utils.generic_utils import Progbar
#from .nets import resnet8 as prediction_network

import keras
from keras.models import Model
from keras.layers import Dense, Dropout, Activation, Flatten, Input
from keras.layers import Conv2D, MaxPooling2D, GlobalAveragePooling2D
from keras.layers.merge import add, concatenate
from keras import regularizers

#from .data_utils import DirectoryIterator

import re
from keras.preprocessing.image import Iterator




# OWN
import colorama
import pprint







#%% from ddr_learner.common_flags import FLAGS


FLAGS = gflags.FLAGS

# gflags.DEFINE_XXX( name, default, help )

# Train parameters
gflags.DEFINE_integer(  'img_width',        300,    'Target Image Width'    )
gflags.DEFINE_integer(  'img_height',       200,    'Target Image Height')
gflags.DEFINE_integer(  'batch_size',       32,     'Batch size in training and evaluation')
gflags.DEFINE_integer(  'output_dim',       3,      "Number of output dimensionality")

gflags.DEFINE_float(    "learning_rate",    0.001,  "Learning rate of for adam")
gflags.DEFINE_float(    "beta1",            0.9,    "Momentum term of adam")
gflags.DEFINE_float(    "f",                1.0,    "Model Width, float in [0,1]")

gflags.DEFINE_string(   'train_dir',        "../../data/Training",      'Folder containing training experiments')
gflags.DEFINE_string(   'val_dir',          "../../data/Testing",       'Folder containing validation experiments')
gflags.DEFINE_string(   'checkpoint_dir',   "/tmp/debug_learning",      "Directory name to save checkpoints and logs.")

# Input Queues reading
gflags.DEFINE_integer(  'num_threads',      8,      'Number of threads reading and (optionally) preprocessing input files into queues')
gflags.DEFINE_integer(  'capacity_queue',   100,    'Capacity of input queue. A high number speeds up computation but requires more RAM')

# Log parameters
gflags.DEFINE_bool(     'resume_train',     False,  'Whether to restore a trained model for training')

gflags.DEFINE_integer(  "max_epochs",       100,    "Maximum number of training epochs")
gflags.DEFINE_integer(  "summary_freq",     100,    "Logging every log_freq iterations")
gflags.DEFINE_integer(  "save_latest_freq", 100,    "Save the latest model every save_latest_freq iterations (overwrites the previous latest model)")

# Testing parameters
gflags.DEFINE_integer(  'test_img_width',   300,    'Target Image Width')
gflags.DEFINE_integer(  'test_img_height',  200,    'Target Image Height')

gflags.DEFINE_string(   'test_dir',         "../../data/validation_sim2real/beauty",    'Folder containing testing experiments')
gflags.DEFINE_string(   'output_dir',       "./tests/test_0",                           'Folder containing testing experiments')
gflags.DEFINE_string(   "ckpt_file",        "/tmp/logs/model-2",                        "Checkpoint file")



#%% from Network import Network


CVBridge = CvBridge()

'''
class Network(object):
    def __init__(self, config): # config=FLAGS

        self.config = config

        self.ROSPub_CNN_Output = rospy.Publisher("cnn_predictions", TwistStamped, queue_size=1)
        
        self.ROSSub_HummingbirdStateChange  = rospy.Subscriber("/hummingbird/state_change", Bool,   self.ROSCallback_HummingbirdStateChange,    queue_size=1)
        self.ROSSub_Checkpoint              = rospy.Subscriber("/checkpoint",               String, self.ROSCallback_Checkpoint,                queue_size=1)
        self.ROSSub_Gamma                   = rospy.Subscriber("/gamma",                    String, self.ROSCallback_Gamma,                     queue_size=1)

        self.learner = TrajectoryLearner()
        self.learner.setup_inference( config, mode='prediction' )

        self.saver = tf.train.Saver( [var for var in tf.trainable_variables()] )
        self.use_network_out = False
        self.smoothed_pred = np.zeros((3,))
        self.alpha = 1.0
        self.checkpoint = ""
        self.gamma = ""

    def ROSCallback_HummingbirdStateChange( self, data ):
        self.use_network_out = data.data
        
        if self.use_network_out:
            if self.config.ckpt_file:
                checkpoint = self.config.ckpt_file
            else:
                checkpoint = tf.train.latest_checkpoint( self.config.checkpoint_dir )
            
            self.saver.restore( self.TFSession, checkpoint )
            print("--------------------------------------------------")
            print("Restored checkpoint file {}".format(checkpoint))
            print("--------------------------------------------------")

    
    def ROSCallback_Gamma(self, data):
        self.gamma = data.data


    def ROSCallback_Checkpoint(self, data):
        self.config.ckpt_file = self.config.checkpoint_dir + self.gamma + "/" + data.data


    def run(self, TFSession):
        self.TFSession = TFSession
        while not rospy.is_shutdown():
            data_camera = None

            while data_camera is None:
                try:
                    data_camera = rospy.wait_for_message("camera", Image)
                except:
                    print("could not aquire image data")
                    break

            if self.use_network_out:
                print("Using network prediction!")
            else:
                print("Not using prediction!")
                continue

            # Reading image and processing it through the network
            try:
                cv_input_image = CVBridge.imgmsg_to_cv2( data_camera )

            except CvBridgeError as e:
                print(e)
                continue

            inputs = {}

            cv_input_image = cv2.resize( cv_input_image, (300, 200), interpolation=cv2.INTER_LINEAR )

            inputs[ 'images' ] = cv_input_image[ None ]
            results = self.learner.inference( inputs, TFSession )
            predictions = np.squeeze( results['predictions'] )
            
            self.smoothed_pred = (1 - self.alpha) * self.smoothed_pred + self.alpha * predictions

            CNN_Output = TwistStamped()
            CNN_Output.header.stamp = rospy.Time.now()
            CNN_Output.twist.linear.x = self.smoothed_pred[ 0 ]
            CNN_Output.twist.linear.y = self.smoothed_pred[ 1 ]
            CNN_Output.twist.linear.z = self.smoothed_pred[ 2 ]
            self.ROSPub_CNN_Output.publish( CNN_Output )

'''




######
# 
# 
# 
#     

# TensorFlow and tf.keras
import tensorflow as tf

# Helper libraries
import numpy as np
import matplotlib.pyplot as plt

#print(tf.__version__)

import numpy as np
import os
import PIL
import PIL.Image
import tensorflow as tf
#import tensorflow_datasets as tfds
import rospkg
import pathlib
import random











class ForgetfulANN:
    
    def __init__( self, FLAGS ):
        self.img_width          = FLAGS.img_width
        self.img_height         = FLAGS.img_height
        self.batch_size         = FLAGS.batch_size
        self.output_dim         = FLAGS.output_dim
        self.learning_rate      = FLAGS.learning_rate
        self.beta1              = FLAGS.beta1
        self.f                  = FLAGS.f
        self.train_dir          = FLAGS.train_dir
        self.val_dir            = FLAGS.val_dir
        self.checkpoint_dir     = FLAGS.checkpoint_dir
        self.num_threads        = FLAGS.num_threads
        self.capacity_queue     = FLAGS.capacity_queue
        self.resume_train       = FLAGS.resume_train
        self.max_epochs         = FLAGS.max_epochs
        self.summary_freq       = FLAGS.summary_freq
        self.save_latest_freq   = FLAGS.save_latest_freq
        self.test_img_width     = FLAGS.test_img_width
        self.test_img_height    = FLAGS.test_img_height
        self.test_dir           = FLAGS.test_dir
        self.output_dir         = FLAGS.output_dir
        self.ckpt_file          = FLAGS.ckpt_file

        # Needs more thinking about
        self.AUTOTUNE = tf.data.AUTOTUNE
        self.validation_share = 0.2
        rospack = rospkg.RosPack()
        self.forgetful_drone_dir = pathlib.Path( rospack.get_path('forgetful_drones') )
        
        # Empty ones that are later filled
        self.simulation_dir = pathlib.Path('')
        self.all_run_dirs = []
        self.failed_run_dirs = []
        self.successful_run_dirs = []
        self.image_files = np.array( [] ).astype( np.str )
        self.labels = tf.ragged.constant( [] )
        self.dataset = tf.data.Dataset.from_tensor_slices( self.image_files )
        self.training_ds = self.dataset.skip( 0 )
        self.validation_ds = self.dataset.take( 0 )

        self.checkpoint_dir     = self.forgetful_drone_dir / "tensorflow_model" / self.checkpoint_dir
        if not os.path.exists( self.checkpoint_dir ):
            os.makedirs( self.checkpoint_dir )



    def create_tensorflow_dataset_from_simulation_data( self ):
        
        def set_simulation_dir_path( self ):

            # Set path to "training_data" directory in ros package "forgetful_drones"
            training_data_dir = self.forgetful_drone_dir / "training_data"
            
            # Set path to a simulation in the "training_data" directory
            #   (Specify desired simulation in command-line arguments)
            if self.train_dir == "latest":
                print( f"Data source: latest simulation." )
                simulation_dir = training_data_dir/(
                    sorted([ x for x in training_data_dir.iterdir() if x.is_dir() ])[ -1 ]
                    )
            else:
                print( f"Data source: user-specified simulation." )
                simulation_dir = training_data_dir/self.train_dir

            # Check if path exists
            if simulation_dir.is_dir():
                print( f"  Directory: {simulation_dir}" )
            else:
                print( f"  Error: directory \"{simulation_dir}\" does not exist." )
                simulation_dir = pathlib.Path('')

            self.simulation_dir = simulation_dir


        def set_run_dir_paths( self ):
        
            # Paths to and count of all run directories in simulation dir
            all_run_dirs = sorted([ self.simulation_dir/x for x in self.simulation_dir.iterdir() if x.is_dir() ])
            all_run_count = len( all_run_dirs )

            # Paths to and count of all failed run directories in simulation dir
            with open( str(self.simulation_dir/"fails.txt") ) as file:
                failed_run_dirs = [ "run_"+x.rstrip().zfill(4) for x in file.readlines()]
            failed_run_dirs = [ self.simulation_dir/x for x in failed_run_dirs ]
            failed_run_count = len( failed_run_dirs )

            successful_run_dirs = sorted([ x for x in all_run_dirs if x not in failed_run_dirs ])
            successful_run_count = len( successful_run_dirs )
            
            print( f"  Content:" )
            print( f"    #{all_run_count} runs: [\'{all_run_dirs[0].name}\', ..., \'{all_run_dirs[-1].name}\']" )
            print( f"      of which #{failed_run_count} failed: {[x.name for x in failed_run_dirs]}" )
            print( f"  Use only data from #{successful_run_count} successful runs:" )

            self.all_run_dirs = all_run_dirs
            self.failed_run_dirs = failed_run_dirs
            self.successful_run_dirs = successful_run_dirs


        def set_image_file_paths( self ):

            # Iterate through run directories and store all paths of *.jpg files in list. 
            image_files = []
            for dir in self.successful_run_dirs:
                image_files += list( dir.glob('images/*.jpg') )
                #image_files += sorted(list( dir.glob('images/*.jpg') ))
            image_count = len( image_files )

            print( f"    #{image_count} images in total." )
            image_rand_file = str( random.choice(image_files) )
            #print( f"  Show randomly chosen image: {image_rand_file}." )
            #PIL.Image.open( image_rand_file ).show()

            self.image_files = np.array( image_files ).astype( np.str )
        

        def set_labels( self ):

            labels = []
            for dir in self.all_run_dirs:
                if dir in self.failed_run_dirs:
                    run_labels = []
                else:
                    run_labels = np.genfromtxt( dir/"labels.txt", delimiter=';' )[ :, :3 ]
                labels.append( run_labels )
            
            print( f"  Use #{3} labels per image." )
            
            self.labels = tf.ragged.constant( labels )
            






        # Adjust printing for this method
        print( colorama.Fore.LIGHTYELLOW_EX ) # yellow font
        np.set_printoptions( precision=4 ) 

        print("\n\n+++ Create tf.data.Dataset from simulation data +++\n")

        set_simulation_dir_path( self )
        set_run_dir_paths( self )
        set_image_file_paths( self )
        set_labels( self )



        # Create tf dataset from image file paths
        print( f"\nCreate dataset." )
        self.dataset = tf.data.Dataset.from_tensor_slices( self.image_files )
        print( f"  {self.dataset}" )
        print( f"    Cardinality: {tf.data.experimental.cardinality(self.dataset).numpy()}")



        print( f"\nPreprocessing dataset:" )
        
        print( f"  1) Shuffling:")
        print( f"       First 5 elements of dataset before:")
        for element in self.dataset.take( 5 ):
            print( f"         {element.numpy()}" )
        # shuffle dataset with buffer containing all elements -> perfect shuffling
        self.dataset = self.dataset.shuffle( len(self.image_files) )
        print( f"       First 5 elements of dataset after:")
        for element in self.dataset.take( 5 ):
            print( f"         {element.numpy()}" )
        
        
        print( f"  2) Mapping:")
        print( f"       Type specification of an element before:")
        print( f"         {self.dataset.element_spec}" )

        def get_label( file_path ):
            # Convert the path to a list of path components and take 
            split_file_path = tf.strings.split( file_path, os.path.sep )
            #   third last part (=run directory name)
            #   last part (=file name)
            run_idx = split_file_path[ -3 ]
            run_idx = tf.strings.split( run_idx, "_" )[ -1 ]
            run_idx = tf.strings.to_number( run_idx, tf.int32 )

            image_idx = split_file_path[ -1 ]
            # Remove file extension from file name
            image_idx = tf.strings.split( image_idx, "." )[ 0 ]
            # Get image index from file name
            image_idx = tf.strings.split( image_idx, "_" )[ -1 ]
            image_idx = tf.strings.to_number( image_idx, tf.int32 )

            # Return the corresponding label
            #run_labels = labels[ run_idx ]
            run_labels = tf.gather( self.labels, run_idx )
            label = tf.gather( run_labels, image_idx )
            
            return tf.gather( label, [0, 1, 2] )
            #return label

        def decode_img( file_path ):
            # Load the raw data from the file as a string
            img = tf.io.read_file( file_path )
            # Convert the compressed string to a 3D uint8 tensor
            img = tf.io.decode_jpeg( img, channels=3 )
            # Resize the image to the desired size
            return tf.image.resize( img, [self.img_height, self.img_width] )

        def process_path( file_path ):
            return decode_img( file_path ), get_label( file_path )

        self.dataset = self.dataset.map( 
            process_path, 
            num_parallel_calls=self.AUTOTUNE 
        )

        print( f"       Type specification of an element after:")
        print( f"         {self.dataset.element_spec}" )

        #for image, label in dataset.take(1):
        #    print( f"\tExamplary preprocessed element")
        #    print("\t\tImage shape: ", image.numpy().shape )
        #    print("\t\tLabel: ", label.numpy() )


        print( f"  3) Training-Validation-Devision:")
        print( f"       Cardinality before: {tf.data.experimental.cardinality(self.dataset).numpy()}")
        print( f"         Validation share: {self.validation_share}")
        validation_size = int( len(self.image_files) * self.validation_share )
        self.training_ds = self.dataset.skip( validation_size )
        self.validation_ds = self.dataset.take( validation_size )
        print( f"       Cardinality after:")
        print( f"         Training data set: {tf.data.experimental.cardinality(self.training_ds).numpy()}")
        print( f"         Validation data set: {tf.data.experimental.cardinality(self.validation_ds).numpy()}")
        

        print( f"  4) Configuring for performance:")
        print( f"       - Caching")
        print( f"       - Batching")
        print( f"       - Prefetching")
        # Configure dataset for performance
        def configure_for_performance( ds ):
            ds = ds.cache()
            ds = ds.batch( self.batch_size, drop_remainder=True )
            ds = ds.prefetch( buffer_size=self.AUTOTUNE )
            return ds

        self.training_ds = configure_for_performance( self.training_ds )
        self.validation_ds = configure_for_performance( self.validation_ds )
        
        
        print( f"\nPlotting first 9 images and labels of training dataset." )
        
        image_batch, label_batch = next( iter(self.training_ds) )

        plt.figure( figsize=(10, 10) )
        for i in range( 9 ):
            ax = plt.subplot( 3, 3, i +1 )
            plt.imshow( image_batch[ i ].numpy().astype("uint8") )
            label = label_batch[ i ]
            plt.title( "Label: " + str(label.numpy()) )
            plt.axis("off")
            plt.scatter(
                x=( int ) ( self.img_width / 2 * (1 + label.numpy()[0]) ), 
                y=( int ) ( self.img_height / 2 * (1 - label.numpy()[1]) ), 
                c='r', 
                s=100
            )
        plt.show()

        
        # Reset: printing everything in yellow.
        print( colorama.Style.RESET_ALL )


    def train_ANN():
        pass




# Utility function to parse flags
def parse_gflags( argv ):
    try:
        FLAGS( argv )

        pp = pprint.PrettyPrinter()
        print_flags_dict = {}
        for key in FLAGS.__flags.keys():
            print_flags_dict[ key ] = getattr( FLAGS, key )
        print("\n+++ gflags +++")
        print( "\nFlagValues:\n" )
        pp.pprint( print_flags_dict )
        print()

    except gflags.FlagsError:
      print(f"{colorama.Fore.RED}Usage:{colorama.Style.RESET_ALL}{FLAGS}")
      sys.exit( 2 ) # Exit from Python with exit status 2 for command line syntax errors



if __name__ == "__main__":

    parse_gflags( sys.argv ) # sys.argv: list that contains the command-line arguments passed to the script
    
    forgetful_ANN = ForgetfulANN( FLAGS )
    forgetful_ANN.create_tensorflow_dataset_from_simulation_data()
