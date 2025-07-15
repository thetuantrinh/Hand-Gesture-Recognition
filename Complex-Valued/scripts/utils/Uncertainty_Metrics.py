# -*- coding: utf-8 -*-
"""
Created on Mon Apr 10 11:54:34 2023

@author: tuant
"""
import tensorflow as tf
import tensorflow_probability as tfp

#%%
class Uncertainty_Metrics(object):
    def __init__(self, x_val, y_val, logits):
        super().__init__()
        self.x_val = x_val
        self.y_val = y_val
        self.logits = logits
        self.y_probs = tf.nn.softmax(
            self.logits, axis=-1)

    def _ECE(self, num_bins = 10):
        """Expected Calibration Error (ECE)."""
        y_pred = tf.argmax(
            self.y_probs,
            axis=-1,
            output_type = 'int32'
            )
        y_val = tf.cast(self.y_val, dtype = tf.int32)
        ece = tfp.stats.expected_calibration_error(
            num_bins = num_bins,
            logits=self.logits,
            labels_true=y_val,
            labels_predicted=y_pred
        )
        
        return ece
    
    def _NLL(self, axis=-1):
        """Negative log-likelihood."""
        model_output = tfp.distributions.Categorical(
            logits=self.logits)
        log_prob = model_output.log_prob(self.y_val)
        nll = -tf.reduce_mean(log_prob, axis=axis)
        
        return nll