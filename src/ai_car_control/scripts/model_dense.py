from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import keras
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Input, Conv2D, Conv2DTranspose
from keras.layers import ZeroPadding2D, BatchNormalization, Activation
from keras.layers import UpSampling2D
from keras.optimizers import RMSprop
from keras.callbacks import ModelCheckpoint, LambdaCallback
from keras.models import load_model, Model
from keras.layers.pooling import MaxPooling2D
from keras.utils import plot_model

import numpy as np


class CNN(object):
    def __init__(self, settings):
        self.settings = settings
        self.xdim = self.settings.xdim
        self.ydim = self.settings.ydim
        self.channels = self.settings.channels
        self.model = None

    def build_model(self, lr=1e-3):
        dropout = 0.2
        shape=(None, self.ydim, self.xdim, self.channels)
        input = Input(batch_shape=shape)


        # reference
        xin = Conv2D(filters=16,
                        strides=2,
                        kernel_size=4,
                        activation='relu',
                        padding='same')(input)
        xin = BatchNormalization()(xin)


        # skip layer tensor
        skip = Conv2D(filters=1,
                        strides=2,
                        kernel_size=4,
                        padding='same',
                        activation='relu')(input)
        skip = BatchNormalization()(skip)



        # parallel stage
        dilation_rate = 1
        y = xin
        for i in range(2):
            a = Conv2D(filters=32,
                        kernel_size=5,
                        padding='same',
                        activation='relu',
                        dilation_rate=dilation_rate)(xin)
            a = Dropout(dropout)(a)
            y = keras.layers.concatenate([a, y])
            dilation_rate += 1


        # dense interconnection
        dilation_rate = 1
        x = skip
        for i in range(1):
            x = keras.layers.concatenate([x, y])
            y = Conv2D(filters=64,
                        activation='relu',
                        kernel_size=1,
                        padding='same')(y)
            y = BatchNormalization()(y)
            y = Conv2D(filters=16,
                        activation='relu',
                        kernel_size=5,
                        padding='same',
                        dilation_rate=dilation_rate)(y)
            y = BatchNormalization()(y)
            y = Dropout(dropout)(y)
            dilation_rate += 1


        # input image skip connection
        x = keras.layers.concatenate([y, skip])
        y = Conv2D(filters=1,
                        strides=2,
                        activation='relu',
                        kernel_size=4,
                        padding='same')(x)
        y = BatchNormalization()(y)

        # flatten
        y = Flatten()(y)

        # output
        output = Dense(6,
                        activation='linear',
                        use_bias=True)(y)

        # CNN model
        self.model = Model(input,output)
        if self.settings.model_weights:
            print("Loading checkpoint weights from: %s...."  % self.settings.model_weights)
            self.model.load_weights(self.settings.model_weights)
            print('---> SUCCESS')

        self.model.compile(loss='mse',
                           optimizer=RMSprop(lr=lr))
        return self.model
