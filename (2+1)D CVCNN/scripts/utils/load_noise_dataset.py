import os
import numpy as np
import tensorflow as tf
from sklearn.preprocessing import LabelEncoder
from tensorflow.python.ops.numpy_ops import np_config # type: ignore
np_config.enable_numpy_behavior()
AUTOTUNE = tf.data.AUTOTUNE
 
def load_npy_file(file_path, label):
    def npy_loader(file_path):
        npy_data = np.load(file_path)
        # print(npy_data.shape)
        return npy_data.astype(np.float32)
    
    npy_data = tf.numpy_function(
        npy_loader, [file_path], tf.float32)
    
    return npy_data, label
#%%
def load_data(noise_type = "AWGN_SNR_-5",
         batch_size = 32):
        noise_paths = "/work/tuan.tt19010226/RAW_HGR/dataset/ds/noise_ds/"
        noise_data_paths = tf.io.gfile.glob(
            os.path.join(noise_paths + noise_type, '***/**/*.npy'))

        valid_label_names = [os.path.basename(os.path.dirname(noise_data_path))\
                    for noise_data_path in noise_data_paths]
        label_encoder = LabelEncoder()
        label_encoder.fit(valid_label_names)
        valid_label_names = label_encoder.transform(valid_label_names)
        valid_label_names = tf.convert_to_tensor(valid_label_names)
        valid_loader = tf.data.Dataset.from_tensor_slices(
            (noise_data_paths, valid_label_names),
            name="corrupted_validation_dataset"
            )
        return valid_loader.cache()\
                           .shuffle(len(noise_data_paths), reshuffle_each_iteration=False)\
                           .map(load_npy_file, num_parallel_calls=AUTOTUNE)\
                           .batch(batch_size, num_parallel_calls=AUTOTUNE)\
                           .prefetch(buffer_size=AUTOTUNE)
        