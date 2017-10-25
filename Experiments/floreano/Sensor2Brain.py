import numpy as np
import cv2
dna = np.random.randint(2,size=(10,29)) 
@nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))  
@nrp.MapVariable("ideal_wheel_speed", global_key="ideal_wheel_speed", initial_value=[0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("real_wheel_speed", global_key="real_wheel_speed", initial_value=[0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapSpikeSource("neurons", nrp.map_neurons(range(10), lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.Robot2Neuron()
def Sensor2Brain(t, ideal_wheel_speed, real_wheel_speed, camera, neurons):
    bridge = CvBridge()
    dna = np.random.randint(2,size=(10,29))
    if not isinstance(camera.value, type(None)):
        (thresh, im_bw) = cv2.threshold(bridge.imgmsg_to_cv2(camera.value, "mono8"), 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        iws = ideal_wheel_speed.value
        rws = real_wheel_speed.value
        r = np.zeros((10))
        row_counter=0
        for row in dna:
            for n in range(18):
                if row[n+1] == 1:
                    if n == 16:
                        r[row_counter] = r[row_counter] + np.absolute(100.0*iws[0]-rws[0])
                    elif n == 17:
                        r[row_counter] = r[row_counter] + np.absolute(100.0*iws[1]-rws[1])
                    else:
                        r[row_counter] = r[row_counter] + (im_bw.item(n)/6.0) + (im_bw.item(n+1)/3.0) + (im_bw.item(n+2)/3.0) + (im_bw.item(n+3)/6.0)
            row_counter+=1
        for x in range(10):
            neurons[x].rate = r[x]