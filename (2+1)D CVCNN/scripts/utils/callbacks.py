# -*- coding: utf-8 -*-
"""
Created on Mon Apr 10 09:54:29 2023

@author: tuant
"""
import tensorflow as tf
#%%
def get_callbacks(patience = 5):
    """
    Reduce learning rate when a metric has stopped improving.
    https://www.tensorflow.org/api_docs/python/tf/keras/callbacks/ReduceLROnPlateau
    Args:
        patience (int, optional): _description_. Defaults to 5.
    """
    reduce_lr = tf.keras.callbacks.ReduceLROnPlateau(
        monitor='val_loss',
        factor=0.1,
        patience=patience//2, 
        min_lr=1e-9, 
        verbose=1
        )
    # early_stop = EarlyStopping(
        # monitor='val_loss',
        # patience=patience
        # )            
    # checkpoint = tf.keras.callbacks.ModelCheckpoint(
    #     filepath=checkpoint_filepath,
    #     save_weights_only=True,
    #     monitor='val_accuracy',
    #     mode='max',
    #     save_best_only=True
    #     )
    # tensorboard_callback = tf.keras.callbacks.TensorBoard(
    #     log_dir=log_dir,
    #     histogram_freq=1
    #     )
    
    # return [reduce_lr, checkpoint, tensorboard_callback]
    
    return [reduce_lr]