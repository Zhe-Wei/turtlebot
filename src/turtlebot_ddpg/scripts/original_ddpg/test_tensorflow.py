#! /usr/bin/env python


import sys
print(sys.version)
import os
# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf
from tensorflow.python.client import device_lib
print(device_lib.list_local_devices())
print(tf.__version__)
print(tf.test.is_gpu_available())
# print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))
# tf.config.list_physical_devices('GPU')
print(tf.config.experimental.list_logical_devices())

# from keras import backend as K
# K.tensorflow_backend._get_available_gpus()


# Create TensorFlow object called hello_constant
# hello_constant = tf.constant('Hello World!')
# with tf.Session() as sess:
#     devices = sess.list_devices()