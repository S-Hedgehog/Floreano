# -*- coding: utf-8 -*-
"""
Sensory-motor brain containing 20 neurons. First 10 are proprioceptive neurons encoding 10 joints of
Schunk robotic manipulator. Last 10 neurons are central pattern generators for grasping motion.
"""
# pragma: no cover

__author__ = 'Igor Peric'

import hbp_nrp_cle.tf_framework as nrp
import logging
import pyNN.nest as sim
import numpy as np
from pyNN.nest import *


nest.SetKernelStatus({'dict_miss_is_error': False})
logger = logging.getLogger(__name__)

def create_brain():
    """
    Initializes PyNN with the neuronal network that has to be simulated
    """

    nest.ResetKernel()
    sim.setup(timestep=0.1, min_delay=0.1, max_delay=20.0, threads=1, debug=True)
    C_m = 25.0
    g_L = 2.5
    t_m = C_m / g_L

    SENSORPARAMS = {'cm': C_m * 1e-3,
                    'v_rest': -60.5,
                    'tau_m': t_m,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -60.5,
                    'v_thresh': -60.0,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 2.5}

    cells = sim.Population(20, sim.IF_cond_alpha, SENSORPARAMS)

    logger.debug("Circuit description: " + str(cells.describe()))
    return cells


circuit = create_brain()
