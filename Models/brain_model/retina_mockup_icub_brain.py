"""
This file contains the setup of the neural network running the iCub experiment with the retina model
"""

import pyNN.nest as sim
import logging

logger = logging.getLogger(__name__)


def create_brain():
    """
    Initializes PyNN with the neuronal network that has to be simulated
    """
    sim.setup(timestep=0.1, min_delay=0.1, max_delay=20.0, threads=1, debug=True)
    cells = sim.Population(320, sim.IF_curr_exp, {'v_thresh': 50.0})
    sim.initialize(cells, v=cells.get('v_rest'))
    return cells

circuit = create_brain()
sensors = circuit[0:320]
actors = circuit[0:320]
