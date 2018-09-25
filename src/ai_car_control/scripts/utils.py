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

    def get_labels (self):
        return (self.labels/self.labels[:,0,:].max())[:,0,:]

    def load_images(self):
        image_paths = sorted(glob.glob(os.path.join(self.dataset_path, '*.png')))
        images = [cv2.imread(img,0) for img in image_paths]
        self.images = np.array(images)
        self.images = self.images.reshape(np.append(self.images.shape,1))

    def get_training (self):
        training = self.images[:int(np.ceil(self.images.shape[0]*.7))]

        return self.normalize(training)

    def get_test (self):
        test = self.images[int(np.ceil(self.images.shape[0]*.7)):]

        return self.normalize(test)

    def get_all (self):
        test = self.normalize(self.images[:])

        return self.normalize(test)

    def normalize (self, data):
        # normalize and reshape data between 0 and 1
        data = (data/data.max()).astype('uint8')

        return data
