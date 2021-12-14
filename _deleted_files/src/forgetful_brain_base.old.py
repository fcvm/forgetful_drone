

#!/usr/bin/env python


import os
import datetime
import sys
import gflags

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



#%% from ddr_learner.models.base_learner import TrajectoryLearner




#Base model for learning
TEST_PHASE = 0
TRAIN_PHASE = 1

class TrajectoryLearner(object):
    def __init__(self):
        pass

    def read_from_disk(self, inputs_queue):
        """Consumes the inputs queue.
        Args:
            filename_and_label_tensor: A scalar string tensor.
        Returns:
            Two tensors: the decoded images, and the labels.
        """
        pnt_seq = tf.cast(inputs_queue[1], dtype=tf.float32)
        file_content = tf.read_file(inputs_queue[0])
        image_seq = tf.image.decode_jpeg(file_content, channels=3)

        return image_seq, pnt_seq

    def preprocess_image(self, image):
        """ Preprocess an input image
        Args:
            Image: A uint8 tensor
        Returns:
            image: A preprocessed float32 tensor.
        """
        image = tf.image.resize_images(image,
                [self.config.img_height, self.config.img_width])
        image = tf.cast(image, dtype=tf.float32)
        image = tf.divide(image, 255.0)
        return image

    def get_filenames_list(self, directory):
        # Load labels, velocities and image filenames. The shuffling will be done after
        iterator = DirectoryIterator(directory, shuffle=False)
        return iterator.filenames, iterator.ground_truth

    def build_train_graph(self):
        is_training_ph = tf.placeholder(tf.bool, shape=(), name="is_training")
        with tf.name_scope("data_loading"):
            # generate training and validation batches ( we do not need labels)
            train_batch, n_samples_train = self.generate_batches(
                           self.config.train_dir)
            val_batch, n_samples_test = self.generate_batches(
                 self.config.val_dir, validation=True)

            #image_batch, pnt_batch, n_samples = \
            #self.generate_batches(self.config.train_dir)cannot find out where node was launched ros
            pred_pnt = prediction_network(image_batch,
                                          output_dim=self.config.output_dim,
                                          f=self.config.f)


        with tf.name_scope("compute_loss"):
            point_loss = tf.losses.mean_squared_error(labels=pnt_batch[:,:2],
                                                      predictions=pred_pnt[:,:2])

            vel_loss = tf.losses.mean_squared_error(labels=pnt_batch[:, 2],
                                                    predictions=pred_pnt[:,2])

            train_loss = point_loss + 0.1 * vel_loss

        with tf.name_scope("metrics"):
            _, var = tf.nn.moments(pred_pnt, axes=-1)
            std = tf.sqrt(var)
            point_rmse = tf.sqrt(point_loss)
            vel_rmse = tf.sqrt(vel_loss)


        with tf.name_scope("train_op"):
            train_vars = [var for var in tf.trainable_variables()]
            optimizer  = tf.train.AdamOptimizer(self.config.learning_rate,
                                             self.config.beta1)
            self.grads_and_vars = optimizer.compute_gradients(train_loss,
                                                          var_list=train_vars)
            self.train_op = optimizer.apply_gradients(self.grads_and_vars)
            self.global_step = tf.Variable(0,
                                           name='global_step',
                                           trainable=False)
            self.incr_global_step = tf.assign(self.global_step,
                                              self.global_step+1)

        # Collect tensors that are useful later (e.g. tf summary), maybe add
        # images

        self.train_steps_per_epoch = \
            int(math.ceil(n_samples_train/self.config.batch_size))
        self.val_steps_per_epoch = \
            int(math.ceil(n_samples_test/self.config.batch_size))
        self.pred_pnt = pred_pnt
        self.gt_pnt = pnt_batch
        self.point_rmse = point_rmse
        self.vel_rmse = vel_rmse
        self.pred_stds = std
        self.image_batch = image_batch
        self.is_training = is_training_ph
        self.total_loss = train_loss
        self.val_loss_eval = point_loss


    def generate_batches(self, data_dir, validation=False):
        seed = random.randint(0, 2**31 - 1)
        # Load the list of training files into queues
        file_list, pnt_list= self.get_filenames_list(data_dir)
        # Convert to tensors before passing
        inputs_queue = tf.train.slice_input_producer([file_list,
          pnt_list],
          seed=seed,
          shuffle=not validation)

        image_seq, pnt_seq = self.read_from_disk(inputs_queue)
        # Resize images to target size and preprocess them
        image_seq = self.preprocess_image(image_seq)
        # Form training batches
        image_batch, pnt_batch = tf.train.batch([image_seq,
             pnt_seq],
             batch_size=self.config.batch_size,
             # This should be 1 for validation, but makes training significantly slower.
             # Since we are anyway not interested in the absolute value of the metrics, we keep it > 1.
             num_threads=self.config.num_threads,
             capacity=self.config.capacity_queue,
             allow_smaller_final_batch=validation)
        return [image_batch, pnt_batch], len(file_list)

    def collect_summaries(self):

        pnt_error_sum = tf.summary.scalar("point_rmse", self.point_rmse)
        vel_error_sum = tf.summary.scalar("vel_rmse", self.vel_rmse)
        image_sum = tf.summary.image("image", self.image_batch)
        self.step_sum_op = tf.summary.merge([pnt_error_sum, vel_error_sum, image_sum])
        self.validation_error = tf.placeholder(tf.float32, [])
        self.val_error_log = tf.summary.scalar("Validation_Error",
                                               self.validation_error)

    def save(self, sess, checkpoint_dir, step):
        model_name = 'model'
        print(" [*] Saving checkpoint to %s..." % checkpoint_dir)
        if step == 'latest':
            self.saver.save(sess,
                        os.path.join(checkpoint_dir, model_name + '.latest'))
        else:
            self.saver.save(sess,
                        os.path.join(checkpoint_dir, model_name),
                        global_step=step)

    def train(self, config):
        """High level train function.
        Args:
            config: Configuration dictionary
        Returns:
            None
        """
        self.config = config
        self.build_train_graph()
        self.collect_summaries()
        with tf.name_scope("parameter_count"):
            parameter_count = tf.reduce_sum([tf.reduce_prod(tf.shape(v)) \
                                        for v in tf.trainable_variables()])
        self.saver = tf.train.Saver([var for var in \
            tf.trainable_variables()] +  [self.global_step], max_to_keep=100)
        sv = tf.train.Supervisor(logdir=config.checkpoint_dir,
                                 save_summaries_secs=0,
                                 saver=None)

        gpu_config = tf.ConfigProto()
        gpu_config.gpu_options.allow_growth=True
        with sv.managed_session(config=gpu_config) as sess:
            print("Number of params: {}".format(sess.run(parameter_count)))
            if config.resume_train:
                print("Resume training from previous checkpoint")
                checkpoint = tf.train.latest_checkpoint(
                                                config.checkpoint_dir)
                self.saver.restore(sess, checkpoint)

            progbar = Progbar(target=self.train_steps_per_epoch)

            n_epochs = 0

            for step in count(start=1):
                if sv.should_stop():
                    break
                start_time = time.time()
                fetches = { "train" : self.train_op,
                              "global_step" : self.global_step,
                              "incr_global_step": self.incr_global_step
                             }
                if step % config.summary_freq == 0:
                    fetches["vel_rmse"] = self.vel_rmse
                    fetches["pnt_rmse"] = self.point_rmse
                    fetches["stds"] = self.pred_stds
                    fetches["summary"] = self.step_sum_op

                # Runs a series of operations
                results = sess.run(fetches,
                                   feed_dict={self.is_training: True})

                progbar.update(step % self.train_steps_per_epoch)


                gs = results["global_step"]

                if step % config.summary_freq == 0:
                    sv.summary_writer.add_summary(results["summary"], gs)
                    train_epoch = math.ceil( gs /self.train_steps_per_epoch)
                    train_step = gs - (train_epoch - 1) * self.train_steps_per_epoch
                    print("Epoch: [%2d] [%5d/%5d] time: %4.4f/it point_rmse: %.3f " \
                          "vel_rmse: %.6f, point_std: %.6f, vel_std: %.6f"
                       % (train_epoch, train_step, self.train_steps_per_epoch, \
                          time.time() - start_time, results["pnt_rmse"],
                          results["vel_rmse"],
                          np.mean(results["stds"][:2]),
                          results["stds"][2] ))

                if step % self.train_steps_per_epoch == 0:
                    n_epochs += 1
                    actual_epoch = int(gs / self.train_steps_per_epoch)
                    self.save(sess, config.checkpoint_dir, actual_epoch)
                    progbar = Progbar(target=self.train_steps_per_epoch)
                    # Evaluate val accuracy
                    val_error = 0
                    for i in range(self.val_steps_per_epoch):
                        loss = sess.run(self.val_loss_eval, feed_dict={
                            self.is_training: False})
                        val_error += loss
                    val_error = val_error / self.val_steps_per_epoch
                    # Log to Tensorflow board
                    val_sum = sess.run(self.val_error_log, feed_dict ={
                        self.validation_error: val_error})
                    sv.summary_writer.add_summary(val_sum, n_epochs)
                    print("Epoch [{}] Validation Loss: {}".format(
                        actual_epoch, val_error))
                    if (n_epochs == self.config.max_epochs):
                        break


    def build_test_graph(self):
        """This graph will be used for testing. In particular, it will
           compute the loss on a testing set, or for prediction of trajectories.
        """
        image_height, image_width = self.config.test_img_height, \
                                    self.config.test_img_width

        self.num_channels = 3
        input_uint8 = tf.placeholder(tf.uint8, [None, image_height,
                                    image_width, self.num_channels],
                                    name='raw_input')


        input_mc = self.preprocess_image(input_uint8)

        pnt_batch = tf.placeholder(tf.float32, [None, self.config.output_dim],
                                          name='gt_labels')


        with tf.name_scope("trajectory_prediction"):
            pred_pnt = prediction_network(input_mc,
                    output_dim=self.config.output_dim, f=self.config.f)

        with tf.name_scope("compute_loss"):
            point_loss = tf.losses.mean_squared_error(labels=pnt_batch[:,:2],
                                                      predictions=pred_pnt[:,:2])

            vel_loss = tf.losses.mean_squared_error(labels=pnt_batch[:, 2],
                                                    predictions=pred_pnt[:, 2])
            total_loss = point_loss + vel_loss


        with tf.name_scope("metrics"):
            _, var = tf.nn.moments(pred_pnt, axes=-1)
            std = tf.sqrt(var)

        self.inputs_img = input_uint8
        self.pred_pnt = pred_pnt
        self.gt_pnt = pnt_batch
        self.pred_stds = std
        self.point_loss = point_loss
        self.total_loss = total_loss
        self.vel_loss = vel_loss

    def setup_inference(self, config, mode):
        """Sets up the inference graph.
        Args:
            mode: either 'loss' or 'prediction'. When 'loss', it will be used for
            computing a loss (gt trajectories should be provided). When
            'prediction', it will just make predictions (to be used in simulator)
            config: config dictionary. it should have target size and trajectories
        """
        self.mode = mode
        self.config = config
        self.build_test_graph()

    def inference(self, inputs, sess):
        results = {}
        fetches = {}
        if self.mode == 'loss':
            fetches["vel_loss"] = self.vel_loss
            fetches["pnt_loss"] = self.point_loss
            fetches["stds"] = self.pred_stds

            results = sess.run(fetches,
                               feed_dict= {self.inputs_img: inputs['images'],
                                           self.gt_pnt: inputs['gt_labels']})
        if self.mode == 'prediction':
            results['predictions'] = sess.run(self.pred_pnt, feed_dict = {
                self.inputs_img: inputs['images']})

        return results






#%% from .nets import resnet8 as prediction_network


'''

#def resnet8(img_input, output_dim, scope='Prediction', reuse=False, f=0.25):
def prediction_network(img_input, output_dim, scope='Prediction', reuse=False, f=0.25):
    """
    Define model architecture. The parameter 'f' controls the network width.
    """
    img_input = Input(tensor=img_input)

    with tf.variable_scope(scope, reuse=reuse):
        x1 = Conv2D(int(32*f), (5, 5), strides=[2, 2], padding='same')(img_input)
        x1 = MaxPooling2D(pool_size=(3, 3), strides=[2, 2])(x1)

        # First residual block
        x2 = Activation('relu')(x1)
        x2 = Conv2D(int(32*f), (3, 3), strides=[2, 2], padding='same',
                    kernel_initializer="he_normal",
                    kernel_regularizer=regularizers.l2(1e-4))(x2)

        x2 = Activation('relu')(x2)
        x2 = Conv2D(int(32*f), (3, 3), padding='same',
                    kernel_initializer="he_normal",
                    kernel_regularizer=regularizers.l2(1e-4))(x2)

        x1 = Conv2D(int(32*f), (1, 1), strides=[2, 2], padding='same')(x1)
        x3 = add([x1, x2])

        # Second residual block
        x4 = Activation('relu')(x3)
        x4 = Conv2D(int(64*f), (3, 3), strides=[2, 2], padding='same',
                    kernel_initializer="he_normal",
                    kernel_regularizer=regularizers.l2(1e-4))(x4)
        x4 = Activation('relu')(x4)
        x4 = Conv2D(int(64*f), (3, 3), padding='same',
                    kernel_initializer="he_normal",
                    kernel_regularizer=regularizers.l2(1e-4))(x4)

        x3 = Conv2D(int(64*f), (1, 1), strides=[2, 2], padding='same')(x3)
        x5 = add([x3, x4])

        # Third residual block
        x6 = Activation('relu')(x5)
        x6 = Conv2D(int(128*f), (3, 3), strides=[2, 2], padding='same',
                    kernel_initializer="he_normal",
                    kernel_regularizer=regularizers.l2(1e-4))(x6)

        x6 = Activation('relu')(x6)
        x6 = Conv2D(int(128*f), (3, 3), padding='same',
                    kernel_initializer="he_normal",
                    kernel_regularizer=regularizers.l2(1e-4))(x6)

        x5 = Conv2D(int(128*f), (1, 1), strides=[2, 2], padding='same')(x5)
        x7 = add([x5, x6])

        x = Flatten()(x7)
        x = Activation('relu')(x)
        x = Dropout(0.5)(x)
        x = Dense(int(256*f))(x)
        x = Activation('relu')(x)

        # Output channel
        logits = Dense(output_dim)(x)

    return logits







#%%



class DirectoryIterator(Iterator):
    def __init__(self, directory,
            target_size=(224,224), num_channels=3, # target size is width, height  (compatible with opencv)
            batch_size=32, shuffle=False, seed=None, follow_links=False):
        self.directory = directory
        self.target_size = tuple(target_size)
        self.follow_links = follow_links

        self.num_channels = 3
        self.image_shape = (self.target_size[1], self.target_size[0], self.num_channels)

        self.samples = 0

        experiments = []
        for subdir in sorted(os.listdir(directory)):
            if os.path.isdir(os.path.join(directory, subdir)):
                experiments.append(subdir)
        self.num_experiments = len(experiments)
        self.formats = {'jpg', 'png'}

        # Associate each filename with a corresponding label
        self.filenames = []
        self.ground_truth = []

        for subdir in experiments:
            subpath = os.path.join(directory, subdir)
            self._decode_experiment_dir(subpath)

        # Conversion of list into array
        if len(self.ground_truth) > 0:
            self.ground_truth = np.array(self.ground_truth, dtype = np.float32)
            self.with_gt = True
        else:
            self.with_gt = False

        print('Found {} images belonging to {} experiments.'.format(
                self.samples, self.num_experiments))
        super(DirectoryIterator, self).__init__(self.samples,
                batch_size, shuffle, seed)

    def _recursive_list(self, subpath):
        return sorted(os.walk(subpath, followlinks=self.follow_links),
                key=lambda tpl: tpl[0])

    def _decode_experiment_dir(self, dir_subpath):
        labels_filename = os.path.join(dir_subpath, "labels.txt")
        with_gt = False

        if os.path.isfile(labels_filename):
            # Try load labels
            ground_truth = np.loadtxt(labels_filename, usecols=(0,1,2), delimiter=';')
            with_gt=True
        else:
            print("No GT found in {}".format(dir_subpath))

        # Now fetch all images in the image subdir
        image_dir_path = os.path.join(dir_subpath, "images")
        for root, _, files in self._recursive_list(image_dir_path):
            sorted_files = sorted(files,
                    key = lambda fname: int(re.search(r'\d+',fname).group()))
            for frame_number, fname in enumerate(sorted_files):
                is_valid = False
                for extension in self.formats:
                    if fname.lower().endswith('.' + extension):
                        is_valid = True
                        break
                if is_valid:
                    absolute_path = os.path.join(root, fname)
                    self.filenames.append(absolute_path)
                    if with_gt:
                        self.ground_truth.append(ground_truth[frame_number])
                    self.samples += 1

    def _load_img(self, path):
        """
        Load an image. Ans reshapes it to target size

        # Arguments
            path: Path to image file.
            target_size: Either `None` (default to original size)
                or tuple of ints `(img_width, img_height)`.

        # Returns
            Image as numpy array.
        """
        img = cv2.imread(path)
        if self.target_size:
            if (img.shape[1], img.shape[0]) != self.target_size:
                img = cv2.resize(img, self.target_size,
                                 interpolation=cv2.INTER_LINEAR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def next(self):
        """
        Public function to fetch next batch. Note that this function
        will only be used for evaluation and testing, but not for training.

        # Returns
            The next batch of images and labels.
        """
        with self.lock:
            #index_array, start, current_batch_size = next(
            index_array = next(
                    self.index_generator)
            current_batch_size = index_array.shape[0]

        # Image transformation is not under thread lock, so it can be done in
        # parallel
        batch_x = np.zeros((current_batch_size,) + self.image_shape,
                dtype=np.uint8)

        # Build batch of image data
        for i, j in enumerate(index_array):
            fname = self.filenames[j]
            x = self._load_img(os.path.join(fname))
            batch_x[i] = x
        if self.with_gt:
            batch_y = self.ground_truth[index_array]
        else:
            batch_y = None

        return batch_x, batch_y

'''










#%% deep_drone_racing_node.py


    # RUN NETWORK
def run_network():

    #rospy.init_node('deep_drone_racing_learned_traj', anonymous=True) # !!! forgetful_brain

    with tf.Session() as TFSession:
        
        TFSession.run( tf.global_variables_initializer() )
        
        network = Network( FLAGS )
        
        network.run( TFSession )


# Utility function to parse flags
def parse_gflags( argv ):
    try:
      FLAGS( argv )
    except gflags.FlagsError:
      #print ( 'Usage: %s ARGS\\n%s' % ( sys.argv[0], FLAGS ) ) # argv[0]: script filename
      print(f"{colorama.Fore.RED}Usage:{colorama.Style.RESET_ALL}{FLAGS}")
      print( FLAGS )
      sys.exit( 2 ) # Exit from Python with exit status 2 for command line syntax errors


if __name__ == "__main__":
    parse_gflags( sys.argv ) # sys.argv: list which contains the command-line arguments passed to the script
    run_network()
