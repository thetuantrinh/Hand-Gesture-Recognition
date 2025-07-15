#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  7 10:43:23 2024

@author: tuan.tt19010226
"""
import os
import tensorflow as tf # type: ignore
import numpy as np
from sklearn.preprocessing import LabelEncoder # type: ignore
from tensorflow.python.ops.numpy_ops import np_config # type: ignore
np_config.enable_numpy_behavior()
#%%
labels = ['empty', 'counter_clock-wise',
          'clock-wise', 'push-down', 'pull-up',
          'zoom-in', 'zoom-out',
          'to_left', 'to_right', 'unknown']
AUTOTUNE = tf.data.AUTOTUNE
Ns = 64
Nc = 128
#%%
train_folder_path = '/work/tuan.tt19010226/RAW_HGR/dataset/ds/Train' 
valid_folder_path = '/work/tuan.tt19010226/RAW_HGR/dataset/ds/Valid'
#%%
train_file_paths = tf.io.gfile.glob(
    os.path.join(train_folder_path, '***/**/*.npy'))
valid_file_paths = tf.io.gfile.glob(
    os.path.join(valid_folder_path, '***/**/*.npy'))

train_label_names = [os.path.basename(os.path.dirname(train_file_path))\
            for train_file_path in train_file_paths]
valid_label_names = [os.path.basename(os.path.dirname(valid_file_path))\
            for valid_file_path in valid_file_paths]
        
label_encoder = LabelEncoder()
label_encoder.fit(train_label_names)
train_labels = label_encoder.transform(train_label_names)
valid_labels = label_encoder.transform(valid_label_names)
    
tensor_train_file_paths = tf.convert_to_tensor(train_file_paths)
tensor_valid_file_paths = tf.convert_to_tensor(valid_file_paths)
tensor_train_labels = tf.convert_to_tensor(train_labels)
tensor_valid_labels = tf.convert_to_tensor(valid_labels)
#%%
def load_npy_file(file_path, label):
    def npy_loader(file_path):
        npy_data = np.load(file_path)
        # print(npy_data.shape)
        return npy_data.astype(np.float32)
    
    npy_data = tf.numpy_function(
        npy_loader, [file_path], tf.float32)
    
    return npy_data, label
#%%
def load(use_tf_ds = True,
         batch_size = 32):
    if use_tf_ds: ## Move dataset to Tensorflow Dataset
        train_loader = tf.data.Dataset.from_tensor_slices(
            (tensor_train_file_paths, tensor_train_labels),
            name="training_dataset"
            )
        valid_loader = tf.data.Dataset.from_tensor_slices(
            (tensor_valid_file_paths, tensor_valid_labels),
            name="validation_dataset"
            )
    
        return (train_loader.cache()
                            .shuffle(len(train_file_paths), reshuffle_each_iteration=True)
                            .map(load_npy_file, num_parallel_calls=AUTOTUNE)
                            .batch(batch_size, num_parallel_calls=AUTOTUNE)
                            .prefetch(buffer_size=AUTOTUNE)),\
               (valid_loader.cache()
                            .shuffle(len(valid_file_paths), reshuffle_each_iteration=False)
                            .map(load_npy_file, num_parallel_calls=AUTOTUNE)
                            .batch(batch_size, num_parallel_calls=AUTOTUNE)
                            .prefetch(buffer_size=AUTOTUNE))