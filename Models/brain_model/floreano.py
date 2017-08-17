# -*- coding: utf-8 -*-
"""
This file contains the setup of the neuronal network running the Floreano experiment with neuronal image recognition
"""
# pragma: no cover

__author__ = 'Stefan Walke'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np
import logging

logger = logging.getLogger(__name__)


def create_brain():
    """
    Initializes PyNN with the neuronal network that has to be simulated
    """
    SENSORPARAMS = {'v_rest': -60.5,
                    'cm': 0.025,
                    'tau_m': 4.0,
                    'tau_refrac': 2.0,
                    'tau_syn_E': 10.0,
                    'tau_syn_I': 10.0,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_thresh': -60.4,
                    'v_reset': -60.5}

    BRAINPARAMS = {'v_rest': -60.5,
                    'cm': 0.025,
                    'tau_m': 4.,
                    'tau_refrac': 2.0,
                    'tau_syn_E': 10,
                    'tau_syn_I': 10,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_thresh': -60.4,
                    'v_reset': -60.5}
    
    ACTORPARAMS = {'v_rest': -60.5,
                    'cm': 0.025,
                    'tau_m': 4.0,
                    'tau_refrac': 2.0,
                    'tau_syn_E': 10.0,
                    'tau_syn_I': 10.0,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_thresh': -60.4,
                    'v_reset': -60.5}

    SYNAPSE_PARAMS = {"weight": 1,
                      "delay": 2.0,
                      'U': 1.0,
                      'tau_rec': 1.0,
                      'tau_facil': 1.0}

    population = sim.Population(32, sim.IF_cond_alpha())
    population[0:18].set(**SENSORPARAMS)
    population[18:28].set(**BRAINPARAMS)
    population[28:32].set(**ACTORPARAMS)

    # Connect neurons
    CIRCUIT = population

    SYN = sim.TsodyksMarkramSynapse(**SYNAPSE_PARAMS)

    sim.Projection(presynaptic_population=CIRCUIT[0:18],
                   postsynaptic_population=CIRCUIT[18:28],
                   connector=sim.AllToAllConnector(allow_self_connections=False),
                   synapse_type=SYN,
                   receptor_type='excitatory')

    sim.Projection(presynaptic_population=CIRCUIT[18:28],
                   postsynaptic_population=CIRCUIT[18:28],
                   connector=sim.AllToAllConnector(allow_self_connections=True),
                   synapse_type=SYN,
                   receptor_type='excitatory')

    sim.Projection(presynaptic_population=CIRCUIT[18:28],
                   postsynaptic_population=CIRCUIT[28:32],
                   connector=sim.AllToAllConnector(allow_self_connections=False),
                   synapse_type=SYN,
                   receptor_type='excitatory')

    sim.initialize(population, v=population.get('v_rest'))

    logger.debug("Circuit description: " + str(population.describe()))

    return population


circuit = create_brain()

