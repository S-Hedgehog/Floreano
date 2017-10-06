import numpy as np
import cv2

@nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))

dna = np.random.randint(2,size=(10,29))

connectors = [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []]

row_counter=0
for row in dna:
    for i in range(1,19):
        if row[i]==1:
            name = "scon" + str(i) + "to" + str(row_counter)
            @nrp.MapSpikeSource(name, nrp.brain.brain[rowcounter], nrp.poisson)
            connectors[i-1].append(eval(name))
            clientLogger.info(name)
    row_counter+=1
    
connectors = np.array(connectors)
    
@nrp.MapVariable("ideal_wheel_speed", global_key="ideal_wheel_speed", initial_value=[0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("real_wheel_speed", global_key="real_wheel_speed", initial_value=[0.0,0.0], scope=nrp.GLOBAL)

@nrp.Robot2Neuron()

def Sensor2Brain(t, ideal_wheel_speed, real_wheel_speed, camera, connectors:

    connections = np.array(connectors)
    bridge = CvBridge()

    if not isinstance(camera.value, type(None)):

        # Get an OpenCV image converted to a 8-bit greyscale image and to a binary black and white image.
        (thresh, im_bw) = cv2.threshold(bridge.imgmsg_to_cv2(camera.value, "mono8"), 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # Fold the image to get a 16 values used to set the firing rates. 4 pixels are collapsed into one value with a weighted sum (center pixels have double weights)
        for n in range(16):
            r = (im_bw.item(n)/6.0) + (im_bw.item(n+1)/3.0) + (im_bw.item(n+2)/3.0) + (im_bw.item(n+3)/6.0)
            for x in connections[n]
                x.rate = r


    iws = ideal_wheel_speed.value
    rws = real_wheel_speed.value
    for x in connections[16]
        x.rate = np.absolute(100.0*iws[0]-rws[0])
    for y in connections[17]
        y.rate = np.absolute(100.0*iws[1]-rws[1])