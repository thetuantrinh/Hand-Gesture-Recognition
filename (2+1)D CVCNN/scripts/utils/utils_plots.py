import matplotlib.pyplot as plt
import tensorflow as tf
from sklearn.metrics import confusion_matrix, accuracy_score
import numpy as np
import pandas as pd
import seaborn as sn
import datetime
import pickle
import io

# plt.rcParams["font.family"] = "serif"
# plt.rcParams["font.serif"] = ["Times New Roman"]
now = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
def plot_history_loss(history,title = None):
    plt.figure(figsize=(5,4))    
    plt.plot(history['loss'])
    plt.plot(history['val_loss'])
    plt.title(title) 
    plt.legend(('Train', 'Valid'))
    plt.show()

def plot_history(history, title = None, type_plot = 0):
    if type_plot == 0:
        plt.subplots(figsize=(5,4), dpi=400) 
        plt.subplot(2,1,1)
        plt.title(title)
        plt.plot(history['loss'])
        plt.plot(history['val_loss'])
        plt.legend(('Train', 'Valid'))
        plt.show()

        plt.subplots(figsize=(5,4), dpi=400) 
        plt.subplot(2,1,2)
        plt.plot(history['accuracy'])
        plt.plot(history['val_accuracy'])
        # plt.legend(('Train', 'Valid'))
        plt.show()
    else:
        fig, ax1 = plt.subplots(figsize=(8, 7))
        plt.title(title)
        ax1.set_xlabel('Epoch', fontsize=14)
        ax1.set_ylabel('Loss', fontsize=14)
        ax1.plot(history['loss'], '-.', color='red', label="Training loss")
        ax1.plot(history['val_loss'], '-.', color='blue', label="Valid loss")
        plt.legend(loc = 10, fontsize=14)
        ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
        ax2.set_ylabel('Accuracy', fontsize=14)  # we already handled the x-label with ax1
        ax2.plot(history['sparse_categorical_accuracy'], color='red', label="Training acc")
        ax2.plot(history['val_sparse_categorical_accuracy'], color='blue', label="Valid acc")
        fig.tight_layout()  # otherwise the right y-label is slightly clipped
        plt.grid(linestyle = '-.')
        plt.legend(loc = 7, fontsize=14)
        plt.show()

#%% Confusion Matrix %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
def plot_confusion_matrix(true_label, predict_label, labels, pl = False):
    cm = confusion_matrix(y_true = true_label, y_pred = predict_label)
    cm_per = np.round(cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]*100, 1)
    print('Avr. accurac: ', accuracy_score(true_label, predict_label))
    print(cm_per)
    # with open('/home/tuan.tt19010226/model_uncertainty/results/' + 'cf_' + now, 'wb') as file:
    #     pickle.dump(cm_per, file)
    if pl is True:
        tick_marks = np.arange(len(labels))+0.5
        df_cm = pd.DataFrame(cm_per, index = labels, columns = labels)
        print(df_cm)
        plt.figure(figsize = (6, 4))    
        labels = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
        sn.heatmap(
            df_cm, annot=True, cmap = "Blues",
            linewidths=.1 ,fmt='.2f', cbar=False
            )
        plt.yticks(tick_marks, labels, rotation=0, fontsize=13)
        # plt.title("Confusion matrix")
        plt.xlabel('True label', fontsize=13)
        plt.ylabel('Predicted label', fontsize=13)
        plt.tight_layout()
        plt.show()
        
        
        
        sn.heatmap(df_cm, annot=True, cmap = "Blues", linewidths=.1 ,fmt='.2f')
        plt.yticks(tick_marks, labels, rotation=0)
        # plt.title("Confusion matrix")
        plt.xlabel('True label')
        plt.ylabel('Predicted label')
        plt.tight_layout()
        
    return accuracy_score(true_label, predict_label)
        
def viz_confusion_matrix(cm, class_names):
    """
    Returns a matplotlib figure containing the plotted confusion matrix.

    Args:
        cm (array, shape = [n, n]): a confusion matrix of integer classes
        class_names (array, shape = [n]): String names of the integer classes
    """
    figure = plt.figure(figsize = (8, 8))
    tick_marks = np.arange(len(class_names))
    cm_per = np.round(cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]*100, 2)
    df_cm = pd.DataFrame(cm_per, index = class_names, columns = class_names)
    sn.heatmap(df_cm, annot=True, cmap = "Blues", linewidths=.1 , fmt='.2f')
    plt.title("Confusion matrix")
    plt.ylabel('True label')
    plt.xlabel('Predicted label') 
    plt.xticks(tick_marks, class_names, rotation=45)
    plt.yticks(tick_marks, class_names)
    plt.tight_layout()
    
    return figure

def plot_to_image(figure):
    """Converts the matplotlib plot specified by 'figure' to a PNG image and
    returns it. The supplied figure is closed and inaccessible after this call."""
    # Save the plot to a PNG in memory.
    buf = io.BytesIO()
    plt.savefig(buf, format='png')
    # Closing the figure prevents it from being displayed directly inside
    # the notebook.
    plt.close(figure)
    buf.seek(0)
    # Convert PNG buffer to TF image
    image = tf.image.decode_png(buf.getvalue(), channels=4)
    # Add the batch dimension
    image = tf.expand_dims(image, 0)
    
    return image