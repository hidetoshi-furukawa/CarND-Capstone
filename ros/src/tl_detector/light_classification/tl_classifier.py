# -*- coding: utf-8 -*-
"""
Class definition of YOLO_v3 style detection model on image
"""

from __future__ import division
from __future__ import print_function

import os
from timeit import default_timer as timer

import cv2
import numpy as np
from keras import backend as K
from keras.models import load_model
from keras.layers import Input
from PIL import Image

from styx_msgs.msg import TrafficLight
from model import yolo_eval, yolo_body, tiny_yolo_body
from utils import letterbox_image
import os
# from keras.utils import multi_gpu_model

class TLClassifier(object):
    _defaults = {
        "model_path": 'models/sim_model.h5',
        "anchors_path": 'light_classification/yolo_tiny_anchors.txt',
        "classes_path": 'light_classification/tl_classes.txt',
        "score" : 0.3,
        "iou" : 0.45,
        "model_image_size" : (416, 416),
        "gpu_num" : 1,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    def __init__(self, is_site):
        self.__dict__.update(self._defaults) # set up default values
        if is_site:
            self.model_path = 'models/real_model.h5'
            self.anchors_path = 'light_classification/yolo_tiny_anchors.txt'
        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        self.sess = K.get_session()
        self.boxes, self.scores, self.classes = self.generate()

        self.image_count = 0
        self.last_pred = TrafficLight.UNKNOWN

    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names

    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape(-1, 2)

    def generate(self):
        model_path = os.path.expanduser(self.model_path)
        assert model_path.endswith('.h5'), 'Keras model or weights must be a .h5 file.'

        # Load model, or construct model and load weights.
        num_anchors = len(self.anchors)
        num_classes = len(self.class_names)
        is_tiny_version = num_anchors==6 # default setting
        try:
            self.yolo_model = load_model(model_path, compile=False)
        except:
            self.yolo_model = tiny_yolo_body(Input(shape=(None,None,3)), num_anchors//2, num_classes) \
                if is_tiny_version else yolo_body(Input(shape=(None,None,3)), num_anchors//3, num_classes)
            self.yolo_model.load_weights(self.model_path) # make sure model, anchors and classes match
        else:
            assert self.yolo_model.layers[-1].output_shape[-1] == \
                num_anchors/len(self.yolo_model.output) * (num_classes + 5), \
                'Mismatch between model and given anchor and class sizes'

        print('{} model, anchors, and classes loaded.'.format(model_path))

        # Generate output tensor targets for filtered bounding boxes.
        self.input_image_shape = K.placeholder(shape=(2, ))
        if self.gpu_num>=2:
            self.yolo_model = multi_gpu_model(self.yolo_model, gpus=self.gpu_num)
        boxes, scores, classes = yolo_eval(self.yolo_model.output, self.anchors,
                len(self.class_names), self.input_image_shape,
                score_threshold=self.score, iou_threshold=self.iou)
        return boxes, scores, classes

    def get_classification(self, im):
        start = timer()

        image = Image.fromarray(im[..., ::-1])

        if self.model_image_size != (None, None):
            assert self.model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert self.model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = letterbox_image(image, tuple(reversed(self.model_image_size)))
        else:
            new_image_size = (image.width - (image.width % 32),
                              image.height - (image.height % 32))
            boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')

        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })

        max_class = None
        max_score = None
        if out_scores.size > 0:
            max_score_idx = out_scores.argmax()
            max_class = out_classes[max_score_idx]
            max_score = out_scores[max_score_idx]
            # print(max_class, max_score, max_score_idx)

        end = timer()

        # print('Found {} boxes for {}'.format(len(out_boxes), 'img'))
        # for i, score in enumerate(out_scores):
            # print(i, out_classes[i], score)

        if max_class is not None:
            predicted_class = self.class_names[max_class]
            dt = end - start
            print("Found traffic light: {light:%s score:%.3f dt:%.3f}"%(predicted_class, max_score, dt))

        rtn = TrafficLight.UNKNOWN

        if max_class == 0:
            rtn = TrafficLight.RED
        elif max_class == 1:
            rtn = TrafficLight.YELLOW
        elif max_class == 2:
            rtn = TrafficLight.GREEN

        self.last_pred = rtn
        self.image_count += 1

        return rtn

    def close_session(self):
        self.sess.close()
