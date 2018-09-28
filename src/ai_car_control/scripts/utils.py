import numpy as np
import cv2
import os, glob

# Utility class
class Settings(object):
    def __init__(self):
        self.ydim = 0
        self.xdim = 0
        self.channels = 0
        self.model_weights = None

class DataLoader(object):
    def __init__(self, dataset_path):
        self.dataset_path = dataset_path
        self.images = None
        self.labels = None

    def load_labels (self):
        self.labels = np.genfromtxt(os.path.join(self.dataset_path, 'labels.txt'), delimiter=',')
        self.labels = self.labels.reshape(np.append(self.labels.shape,1))
        return (self.labels/180)[:,0,:]

    def load_images(self):
        self.images = np.load(os.path.join(self.dataset_path, 'compressed_data.npz'))['images']
        return self.images


