"""
This file contains the setup of the neural network running the iCub experiment with the retina model
"""

import pyNN.nest as sim
import logging
import math

logger = logging.getLogger(__name__)

def create_brain():
    """
    Initializes PyNN with the neuronal network that has to be simulated
    """
    # layers size
    sz = 320
    # edge detection layer
    d = 7
    # first layer
    cells = sim.Population(4*sz, sim.IF_curr_exp())

    pop = cells[0:4*sz]

    layer1_OFF = cells[0:sz]
    layer1_ON = cells[sz:2*sz]
    layer2_OFF = cells[2*sz:3*sz]
    layer2_ON = cells[3*sz:]

    for i in range(sz):
        sim.Projection(layer1_OFF[max(0, i-(d/2)):min(i+(d/2), 319)+1], layer2_OFF[i:i+1],
                   sim.AllToAllConnector(),
                   sim.StaticSynapse(weight=2.0-(abs(float(i-160))/80), delay=0.1),
                   receptor_type='excitatory')

        sim.Projection(layer1_ON[max(0, i-(d/2)):min(i+(d/2), 319)+1], layer2_ON[i:i+1],
                   sim.AllToAllConnector(),
                   sim.StaticSynapse(weight=2.0-(abs(float(i-160))/80), delay=0.1),
                   receptor_type='excitatory')

    return cells

circuit = create_brain()
