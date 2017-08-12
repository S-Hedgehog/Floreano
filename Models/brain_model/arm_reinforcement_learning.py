# -*- coding: utf-8 -*-
"""
"""
# pragma: no cover

__author__ = 'Martin Schulze'

import nest
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
    # sim.setup(timestep=0.1, min_delay=0.1, max_delay=20.0, threads=2, debug=True)

    INPUT_OUTPUT_PARAMS = {
        # 'v_thresh':      -55.0,
        # 'tau_m':     100.0,
        # 'cm':       100.0,
        # 'v_reset':   -60.0
    }

    DOPAMINE_LAYER_PARAMS = {
        # 'v_thresh':      -55.0,
        # 'tau_m':     100.0,
        # 'cm':       100.0,
        # 'v_reset':   -60.0
    }

    aon_input = 90
    aon_dm = 2
    aon_output = 2

# Create Layers

    neurons = sim.Population(94, sim.IF_cond_alpha, INPUT_OUTPUT_PARAMS)
    input_layer = sim.PopulationView(neurons, slice(0, 90))
    dm_layer = sim.PopulationView(neurons, slice(90, 92))
    dm_layer.set(**DOPAMINE_LAYER_PARAMS)
    output_layer = sim.PopulationView(neurons, slice(92, 94))
    # output_layer.set(i_offset=30.0)

    vt = nest.Create('volume_transmitter', 2)


# Connect Layers

    nest.CopyModel('stdp_dopamine_synapse', 'syn1',
                   {'Wmax': 500.0,
                    'Wmin': 0.0,
                    'tau_plus': 50.0,
                    'A_minus': 150.0,
                    'A_plus': 50.0,
                    'b': 0.01,
                    'tau_c': 10.0,
                    'tau_n': 5.0,
                    'vt': vt[0]})

    nest.Connect([map(int, dm_layer.all_cells)[1]], vt)
    syn_con = nest.Connect(map(int, input_layer.all_cells),
                           map(int, output_layer.all_cells),
                           syn_spec={'model': 'syn1', 'weight': 0.01, 'delay': 1.0})

    # nest.SetStatus(map(int, input_layer.all_cells),
    #                {'V_m': INPUT_OUTPUT_PARAMS['v_reset']})


    # logger.debug("Circuit description: " + str(neurons.describe()))
    return neurons


circuit = create_brain()
