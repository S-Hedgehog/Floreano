import numpy as np
import cv2

@nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))
@nrp.MapSpikeSource("pixel0", nrp.brain.sensors[0], nrp.poisson)
@nrp.MapSpikeSource("pixel1", nrp.brain.sensors[1], nrp.poisson)
@nrp.MapSpikeSource("pixel2", nrp.brain.sensors[2], nrp.poisson)
@nrp.MapSpikeSource("pixel3", nrp.brain.sensors[3], nrp.poisson)
@nrp.MapSpikeSource("pixel4", nrp.brain.sensors[4], nrp.poisson)
@nrp.MapSpikeSource("pixel5", nrp.brain.sensors[5], nrp.poisson)
@nrp.MapSpikeSource("pixel6", nrp.brain.sensors[6], nrp.poisson)
@nrp.MapSpikeSource("pixel7", nrp.brain.sensors[7], nrp.poisson)
@nrp.MapSpikeSource("pixel8", nrp.brain.sensors[8], nrp.poisson)
@nrp.MapSpikeSource("pixel9", nrp.brain.sensors[9], nrp.poisson)
@nrp.MapSpikeSource("pixel10", nrp.brain.sensors[10], nrp.poisson)
@nrp.MapSpikeSource("pixel11", nrp.brain.sensors[11], nrp.poisson)
@nrp.MapSpikeSource("pixel12", nrp.brain.sensors[12], nrp.poisson)
@nrp.MapSpikeSource("pixel13", nrp.brain.sensors[13], nrp.poisson)
@nrp.MapSpikeSource("pixel14", nrp.brain.sensors[14], nrp.poisson)
@nrp.MapSpikeSource("pixel15", nrp.brain.sensors[15], nrp.poisson)
@nrp.MapSpikeSource("left_wheel_speed_error", nrp.brain.sensors[16], nrp.poisson)
@nrp.MapSpikeSource("right_wheel_speed_error", nrp.brain.sensors[17], nrp.poisson)

@nrp.MapVariable("ideal_wheel_speed", global_key="ideal_wheel_speed", initial_value=[0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("real_wheel_speed", global_key="real_wheel_speed", initial_value=[0.0,0.0], scope=nrp.GLOBAL)

@nrp.Robot2Neuron()

def Sensor2Brain(t, ideal_wheel_speed, real_wheel_speed, left_wheel_speed_error, right_wheel_speed_error,camera, pixel0, pixel1, pixel2, pixel3, pixel4, pixel5, pixel6, pixel7, pixel8, pixel9, pixel10, pixel11, pixel12, pixel13, pixel14, pixel15):
    """
    This transfer function uses OpenCV to compute the grayscale values of the camera pixels. Then, it maps these values
    to the neural network's first 16 sensory neurons using a Poisson generator.
    """
    neurons = np.array([pixel0, pixel1, pixel2, pixel3, pixel4, pixel5, pixel6, pixel7, pixel8, pixel9, pixel10, pixel11, pixel12, pixel13, pixel14, pixel15])
    bridge = CvBridge()

    if not isinstance(camera.value, type(None)):

        # Get an OpenCV image converted to a 8-bit greyscale image and to a binary black and white image.
        (thresh, im_bw) = cv2.threshold(bridge.imgmsg_to_cv2(camera.value, "mono8"), 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        # Fold the image to get a 1x16 binary array mask16, where 0 denotes black and 1 denotes white areas.
        for n in range(16):
            r = (im_bw.item(n)/6.0) + (im_bw.item(n+1)/3.0) + (im_bw.item(n+2)/3.0) + (im_bw.item(n+3)/6.0)
            neurons.item(n).rate = r


    iws = ideal_wheel_speed.value
    rws = real_wheel_speed.value
    left_wheel_speed_error.rate = np.absolute(5000*iws[0]-rws[0])
    right_wheel_speed_error.rate = np.absolute(5000*iws[1]-rws[1])