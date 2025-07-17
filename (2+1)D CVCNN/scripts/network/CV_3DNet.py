#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 20 20:06:01 2025

@author: tuan.trinhthe
"""
#%%
import complexnn
import tensorflow as tf
#%%
def CV_Conv2Plus1D(inputs,
                   filters,
                   kernel_size,
                   padding,
                   strides):
    """
    Convolutional layers that first apply the convolution operation
    over the spatial dimensions, and then the temporal dimension.
    """
    # Spatial decomposition
    x = complexnn.conv.ComplexConv3D(
        filters=filters,
        kernel_size=(1, kernel_size[1], kernel_size[2]),
        padding=padding,
        strides=strides)(inputs)
    # Temporal decomposition
    x = complexnn.conv.ComplexConv3D(
        filters=filters,
        kernel_size=(kernel_size[0], 1, 1),
        padding=padding,
        strides=strides)(x)
    
    return x
#%%
def ResidualBlock(inputs,
                  filters,
                  kernel_size):
    x = CV_Conv2Plus1D(inputs,
                       filters=filters,
                       kernel_size=kernel_size,
                       padding='same',
                       strides=1)
    x = complexnn.bn.ComplexBatchNormalization()(x)
    x = tf.keras.layers.Activation('relu')(x)
    x = CV_Conv2Plus1D(x, 
                       filters=filters,
                       kernel_size=kernel_size,
                       padding='same',
                       strides=1)
    x = complexnn.bn.ComplexBatchNormalization()(x)
    
    return x
#%%
def Project(inputs, units):
    """
    Project certain dimensions of the tensor as the data is passed through
    different sized filters and downsampled.
    """
    x = complexnn.conv.ComplexConv3D(filters = units//2,
                                     kernel_size = (1, 1, 2),
                                     padding = 'same',
                                     strides = (1, 1, 1))(inputs)
    x = complexnn.bn.ComplexBatchNormalization()(x)
    
    return x
#%%
def add_residual_block(inputs,
                       filters,
                       kernel_size):
    
    out = ResidualBlock(inputs, filters, kernel_size)
    res = inputs
    if out.shape[-1] != inputs.shape[-1]:
        res = Project(res, out.shape[-1])

    return tf.keras.layers.add([res, out])
#%%
def CV_Net(input_shape  = ([None, 20, 128, 64, 8]),
           output_shape = 10):
    input_layer = tf.keras.Input(shape=(input_shape[1:]))
    x = CV_Conv2Plus1D(
        input_layer,
        filters=4,
        kernel_size=(3, 3, 2),
        padding='same',
        strides=(1, 2, 1))
    x = complexnn.bn.ComplexBatchNormalization()(x)
    x = tf.keras.layers.Activation('relu')(x)
    # Block 1
    x = add_residual_block(x, 4, (3, 3, 2))
    # Block 2
    x = add_residual_block(x, 4, (3, 3, 2))
    # Block 3
    x = add_residual_block(x, 8, (3, 3, 2))
    # Block 4
    x = add_residual_block(x, 8, (3, 3, 2))
    x = tf.keras.layers.MaxPool3D(pool_size=(2, 2, 2))(x)
    x = tf.keras.layers.Activation('relu')(x)
    x = tf.keras.layers.GlobalAveragePooling3D()(x)
    dropout_out = tf.keras.layers.Dropout(0.05)(x, training=True)
    output_layer = tf.keras.layers.Dense(output_shape)(dropout_out)

    model =  tf.keras.Model(inputs=input_layer,
                            outputs=output_layer,
                            name = 'CV-3DNet')
    print("Num layer: ", len(model.trainable_variables))
    model.summary()
    
    return model
