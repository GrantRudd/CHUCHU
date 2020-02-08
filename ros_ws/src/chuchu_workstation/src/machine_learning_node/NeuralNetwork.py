#!/usr/bin/env python

import random, time
import numpy as np
import tensorflow as tf
import os

class NeuralNetwork:
    def __init__(self, _n_nodes_hl1, _n_nodes_hl2, _n_nodes_hl3, _batch_size):
        self._n_nodes_hl1 = _n_nodes_hl1
        self._n_nodes_hl2 = _n_nodes_hl2
        self._n_nodes_hl3 = _n_nodes_hl3
        self._batch_size = _batch_size
        self._n_classes = 2
        self._trainingFeatures = np.array([])
        self._trainingLabels = np.array([])
        self._testFeatures = np.array([])
        self._testLabels = np.array([])
        self._trainingSet = ""
        self._savedParams = ""
        #height x width
        self._x = tf.placeholder('float', [None, 33])        #placeholder(dtype, shape)

        self._y = tf.placeholder('float')

        self._hidden_1_layer = {'weights':tf.Variable(tf.random_normal([33, _n_nodes_hl1])), 
                              'biases':tf.Variable(tf.random_normal([_n_nodes_hl1]))}
    
        #dictionary. weights is a tensorflow variable, and that variable is random normal of a shape
        #biases also a tf.variable random normal, same size as hiddenlayer one.

    
        self._hidden_2_layer = {'weights':tf.Variable(tf.random_normal([_n_nodes_hl1, _n_nodes_hl2])),
                              'biases':tf.Variable(tf.random_normal([_n_nodes_hl2]))}
    
        self._hidden_3_layer = {'weights':tf.Variable(tf.random_normal([_n_nodes_hl2, _n_nodes_hl3])),
                              'biases':tf.Variable(tf.random_normal([_n_nodes_hl1]))}
    
        self._output_layer = {'weights':tf.Variable(tf.random_normal([_n_nodes_hl3, self._n_classes])),
                              'biases':tf.Variable(tf.random_normal([self._n_classes]))}
        self.saver= tf.train.Saver()

        # Disable AVX2 FMA
        os.environ['TF_CPP_MIN_LOG_LEVEL'] ='2'
        
        tf.logging.set_verbosity(tf.logging.FATAL)

    def load_training_set(self, trainingSet):
        basePath = os.path.abspath('')

        # Update training set name
        self._trainingSet = "/home/grant/ros_ws/src/chuchu_workstation/src/machine_learning_node/training_sets/" + trainingSet + '.txt'

        # Update saved file name
        self._savedParams = "/home/grant/ros_ws/src/chuchu_workstation/src/machine_learning_node/tmp/" + trainingSet + ".ckpt"

        count = 0;

        with open(self._trainingSet) as openFileobject:
            while True:
                feature = openFileobject.readline()
                label = openFileobject.readline() #reads line then increments

                if feature == '' or label == '':
                    print "%i Training examples loaded" % count
                    break

                randomNumber = random.randint(1, 100)
                if(randomNumber <= 80):   
                    self._trainingFeatures,self._trainingLabels = self.addToSet(feature, label, self._trainingFeatures, self._trainingLabels)          #places data into either test or training sets
                else:
                    self._testFeatures,self._testLabels = self.addToSet(feature, label, self._testFeatures, self._testLabels)

                count += 1

    def load_parameters(self, parameters):
        basePath = os.path.abspath('')

        # Update saved file name
        self._savedParams = "/home/grant/ros_ws/src/chuchu_workstation/src/machine_learning_node/tmp/" + parameters + ".ckpt"

        with tf.Session() as sess:
            sess.run(tf.global_variables_initializer())
            try:
                self.saver.restore(sess,self._savedParams)
                print "Session '%s' successfully restored" % parameters
                return True
            except:
                return False



    def feedForward(self, data):
        #(input_data * weights)+biases

        l1 = tf.add(tf.matmul(data, self._hidden_1_layer['weights']), self._hidden_1_layer['biases'])   # lowercase L and 1 look identical in idle, l1 stands for Layer one
        l1 = tf.nn.relu(l1)  #applies activation function to layer one. relu means rectified linear unit.
    
        l2 = tf.add(tf.matmul(l1, self._hidden_2_layer['weights']), self._hidden_2_layer['biases'])
        l2 = tf.nn.relu(l2)

        l3 = tf.add(tf.matmul(l2, self._hidden_3_layer['weights']), self._hidden_3_layer['biases'])
        l3 = tf.nn.relu(l3)

        output = tf.matmul(l3, self._output_layer['weights']) + self._output_layer['biases']
        
        return output

    def train_neural_network(self):
        prediction = self.feedForward(self._x)
    
        #basically this calculates the difference between the prediction and known labels
        #cost = tf.reduce_mean( tf.nn.softmax_cross_entropy_with_logits(logits=prediction, labels=y) )
        cost = tf.reduce_mean(tf.nn.weighted_cross_entropy_with_logits(logits=prediction, targets=self._y, pos_weight=tf.constant([1.0, 1.0])))

        # AdamOptimizer has parameter of learning rate, but defaults to 0.001 if unspecified
        optimizer = tf.train.AdamOptimizer(.001).minimize(cost)

        #epoch is a cycle of feed forward + backpropagation
        hm_epochs = 300

        with tf.Session() as sess:
            sess.run(tf.global_variables_initializer())
  
            for epoch in range(hm_epochs):
                epoch_loss = 0
                for _ in range(10):
                    #chunks through dataset
                    epoch_x,epoch_y = self.getBatch(self._trainingFeatures, self._trainingLabels,self._batch_size)
                    #epoch_x = batch of samples
                    #epoch_y = batch of labels
                    _, c = sess.run([optimizer, cost], feed_dict = {self._x: epoch_x, self._y: epoch_y})
                    epoch_loss +=c
                print('epoch', epoch, 'completed out of', hm_epochs, 'loss:', epoch_loss)
            
            correct = tf.equal(tf.argmax(prediction,1), tf.argmax(self._y,1))

            accuracy = tf.reduce_mean(tf.cast(correct, 'float'))
            print('Accuracy:', accuracy.eval({self._x:self._testFeatures, self._y:self._testLabels}))
            self.saver.save(sess, self._savedParams)  #saves trained variables to tmp folder
            
    def softmax(self, x):
        return np.exp(x) / np.sum(np.exp(x), axis=0)
            
    def get_prediction(self, input_data):
        prediction = self.feedForward(self._x)

        with tf.Session() as sess:
            sess.run(tf.global_variables_initializer())
            
            
            try:
                self.saver.restore(sess,self._savedParams)
            except:
                pass
                  
            result = (sess.run(tf.argmax(prediction.eval(feed_dict={self._x:[input_data]}),1))) 
            pred = prediction.eval(feed_dict={self._x:[input_data]}, session = sess)
            true = pred[0][0]    #naming here is very confusing, sorry about that
            false = pred[0][1]
            if(true <false):
                true1 = true/abs(false)
                false1 = false/abs(false)
            else:
                true1 = true/abs(true)
                false1 = false/abs(true)

            prob = self.softmax([true1, false1])
            # Return confidence of good grasp (0.0 -1.0)
            return(prob[1])

    def addToSet(self, line, label, features, labels):
        x1= np.array([float(i) for i in line.split(',')])
        features = np.vstack([features, x1]) if features.size else x1
        y1= np.array([float(i) for i in label.split(',')])
        labels = np.vstack([labels, y1]) if labels.size else y1
        return(features,labels)
        
    def getBatch(self, trainingFeatures, trainingLabels,sizeOfSet):
        batchFeatures = np.array([])
        batchLabels = np.array([])
        for i in range(0,sizeOfSet):
            randomNumber = random.randint(0, len(trainingFeatures)-1)
            batchFeatures= np.vstack([batchFeatures, trainingFeatures[randomNumber]]) if batchFeatures.size else trainingFeatures[randomNumber]
            batchLabels= np.vstack([batchLabels, trainingLabels[randomNumber]]) if batchLabels.size else trainingLabels[randomNumber]
        return(batchFeatures, batchLabels)