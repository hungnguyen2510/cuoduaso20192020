import tensorflow as tf
import numpy as np
import cv2, os, time
import util

class CNN(object):

    def __init__(self):

        # The file path of model
        self.model_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'model', 'tfs_64-1.00.pb')
        # Initialize the model
        self.load_graph()
        self.debug = 1
        self.low_range = np.array([81, 159, 30])
        self.high_range = np.array([131, 255, 175])
        self.kernel = np.ones((3,3), np.uint8)

    def load_graph(self):
        '''
        Lode trained model.
        '''
        print('Loading model...')
        self.graph = tf.Graph()

        with tf.gfile.GFile(self.model_filepath, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())

        print('Check out the input placeholders:')
        nodes = [n.name + ' => ' +  n.op for n in graph_def.node if n.op in ('Placeholder')]
        for node in nodes:
            print(node)

        with self.graph.as_default():
        	# Define input tensor
        	self.input = tf.placeholder(np.float32, shape = [1, 24, 24, 3], name='input')
        	self.dropout_rate = tf.placeholder(tf.float32, shape = [], name = 'dropout_rate')
        	tf.import_graph_def(graph_def, {'input_1': self.input })

        self.graph.finalize()

        print('Model loading complete!')

        # Get layer names
        layers = [op.name for op in self.graph.get_operations()]
        for layer in layers:
            print(layer)
        
        """
        # Check out the weights of the nodes
        weight_nodes = [n for n in graph_def.node if n.op == 'Const']
        for n in weight_nodes:
            print("Name of the node - %s" % n.name)
            # print("Value - " )
            # print(tensor_util.MakeNdarray(n.attr['value'].tensor))
        """

        # In this version, tf.InteractiveSession and tf.Session could be used interchangeably. 
        # self.sess = tf.InteractiveSession(graph = self.graph)
        self.sess = tf.Session(graph = self.graph)

    def predict(self, img):

        # Know your output node name
        output_tensor = self.graph.get_tensor_by_name("import/dense_1/Sigmoid:0")
        output = self.sess.run(output_tensor, feed_dict = {self.input: img, self.dropout_rate: 0})
        return output[0][0]

    def preprocess(self, img):
        img = util.resizeImage(img)
        return np.array(img).reshape(1, 24, 24, 3)

    def extrac_blue(self, frame):
        img, imagePredict = None, None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.low_range, self.high_range)
        if self.debug:
            cv2.imshow('debug-tf-exBlue',mask)
            cv2.waitKey(1)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        # mask = cv2.dilate(mask, self.kernel, iterations=1)
        contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]
        contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]

        if len(contour_sizes) > 0:
            biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
            area = cv2.contourArea(biggest_contour)
            if(area > 400):
                x,y,w,h = cv2.boundingRect(biggest_contour)
                if (abs(w-h) < (w+h)/10):
                    img = frame[y:y+h,x:x+w]
                    imagePredict = util.resizeImage(img)                             
        return imagePredict

    def process(self, img):
        tf_sign = ''
        img = self.extrac_blue(img)
        
        if img is not None:
            if self.debug:
                cv2.imshow('debug-tf',img)
                cv2.waitKey(1)
            img = self.preprocess(img)
            start = time.time()
            predicted = self.predict(img)
            if 0.0 <= predicted <= 0.2:
                tf_sign = 'straight'
            if 0.8 <= predicted <= 1.0:
                tf_sign = 'turn'
            print("{}\t{}\t{}".format(predicted, tf_sign, time.time() - start))
        return tf_sign
