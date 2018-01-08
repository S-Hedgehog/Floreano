import numpy as np
import cv2

@nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))  
@nrp.MapVariable("ideal_wheel_speed", global_key="ideal_wheel_speed", initial_value=[0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("real_wheel_speed", global_key="real_wheel_speed", initial_value=[0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapSpikeSource("r1", nrp.map_neurons(nrp.brain.receptors[0], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r2", nrp.map_neurons(nrp.brain.receptors[1], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r3", nrp.map_neurons(nrp.brain.receptors[2], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r4", nrp.map_neurons(nrp.brain.receptors[3], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r5", nrp.map_neurons(nrp.brain.receptors[4], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r6", nrp.map_neurons(nrp.brain.receptors[5], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r7", nrp.map_neurons(nrp.brain.receptors[6], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r8", nrp.map_neurons(nrp.brain.receptors[7], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r9", nrp.map_neurons(nrp.brain.receptors[8], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r10", nrp.map_neurons(nrp.brain.receptors[9], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r11", nrp.map_neurons(nrp.brain.receptors[10], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r12", nrp.map_neurons(nrp.brain.receptors[11], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r13", nrp.map_neurons(nrp.brain.receptors[12], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r14", nrp.map_neurons(nrp.brain.receptors[13], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r15", nrp.map_neurons(nrp.brain.receptors[14], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r16", nrp.map_neurons(nrp.brain.receptors[15], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r17", nrp.map_neurons(nrp.brain.receptors[16], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.MapSpikeSource("r18", nrp.map_neurons(nrp.brain.receptors[17], lambda i: nrp.brain.brain[i]), nrp.poisson)
@nrp.Robot2Neuron()
def Sensor2Brain(t, ideal_wheel_speed, real_wheel_speed, camera, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15, r16, r17, r18):
    bridge = CvBridge()
    if not isinstance(camera.value, type(None)):
        (thresh, im_bw) = cv2.threshold(bridge.imgmsg_to_cv2(camera.value, "mono8"), 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        iws = np.array(ideal_wheel_speed.value)
        rws = np.array(real_wheel_speed.value)
        visual_receptors = [r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15, r16]
        for i in range(len(visual_receptors)):
            visual_receptors[i].rate = im_bw.item((4*i))/6.0 + im_bw.item((4*i)+1)/3.0 + im_bw.item((4*i)+2)/3.0 + im_bw.item((4*i)+3)/6.0
        for n in range(len(r17)):
            r17[n].rate = 100000*np.absolute(iws[0]-rws[0])
        for n in range(len(r18)):
            r18[n].rate = 100000*np.absolute(iws[1]-rws[1])