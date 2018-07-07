from styx_msgs.msg import TrafficLight
import os
import tensorflow as tf
import numpy as np
import time

class TLClassifier(object):
    def __init__(self):
        # default status
        self.current_light = TrafficLight.UNKNOWN
        #TODO load classifier
        #PATH_TO_MODEL = 'frozen_inference_graph.pb' #load ssd_inception_v2_coco classifier retrained with Bosch traffic light dataset
        #PATH_TO_MODEL = '/mnt/c/users/Aldo/CarND-capstone/ros/src/tl_detector/light_classification/frozen_inference_graph.pb'
        PATH_TO_MODEL = os.path.dirname(os.path.realpath(__file__))+'/frozen_inference_graph_test.pb'
        # Create a label dictionary
        item_green = {'id': 1, 'name': u'GREEN'}
        item_red = {'id': 2, 'name': u'RED'}
        item_yellow = {'id': 3, 'name': u'YELLOW'}

        self.label_dict = {1: item_green, 2: item_red, 3: item_yellow}
        self.detection_graph = tf.Graph()

        # Based mostly in the object detection API tutorial
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            # Works up to here.
            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
            #with tf.gfile.GFile('/frozen_inference_graph.pb', 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.detection_graph)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        with self.detection_graph.as_default():
            # Expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num) = self.sess.run(
                [self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                feed_dict={self.image_tensor: img_expanded})

            #create np arrays
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)

            # Set a Classification threshold
            classification_threshold = .50

            # Iterate the boxes to get all detections
            for i in range(boxes.shape[0]):

                # Get class name for detections with high enough scores
                if scores is None or scores[i] > classification_threshold:
                    class_name = self.label_dict[classes[i]]['name']

                    # Set default state to unknown
                    self.current_light = TrafficLight.UNKNOWN

                    if class_name == 'RED':
                        self.current_light = TrafficLight.RED
                    elif class_name == 'GREEN':
                        self.current_light = TrafficLight.GREEN
                    elif class_name == 'YELLOW':
                        self.current_light = TrafficLight.YELLOW
        # Detected light is assumed to be the closest one
        return self.current_light