# -*- coding: utf-8 -*-
"""
This file contains the setup of the neuronal network running the Mouse experiment with neuronal image recognition
"""
# pragma: no cover

__author__ = 'Lazar Mateev, Georg Hinkel, Alina Roitberg'

### The following can be removed when PyNN 0.8 has been established or we have a more elegant
### solution
from pkg_resources import parse_version
import pyNN

if not parse_version(pyNN.__version__) >= parse_version('0.8.0'):
    raise RuntimeError("The brain model requires PyNN 0.8.0 to be installed. The Platform is "
                       "currently using %s" % pyNN.__version__)
### END: PyNN Version Check

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np
import logging

logger = logging.getLogger(__name__)


def create_brain():
    """
    Initializes PyNN with the neuronal network that has to be simulated for the mouse experiment
    """

    # Following parameters were taken from the husky braitenberg brain experiment (braitenberg.py)

    SENSORPARAMS = {'cm': 0.025,
                    'v_rest': -60.5,
                    'tau_m': 10.0,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -60.5,
                    'v_thresh': -60.0,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 2.5}

    SYNAPSE_PARAMS = {'U': 1.0,
                      'tau_rec': 0.1,
                      'tau_facil': 0.1,
                      'weight': 1.5e-4,
                      'delay': 0.1}

    cell = sim.IF_cond_alpha(**SENSORPARAMS)
    # Defining the network structure: 4 neurons (2 sensors and 2 actors)
    population = sim.Population(4, cell)

    SYN = sim.TsodyksMarkramSynapse(**SYNAPSE_PARAMS)

    # Connect neurons
    CON = sim.AllToAllConnector()

    sim.Projection(presynaptic_population=population[0:1],
                   postsynaptic_population=population[2:3],
                   connector=CON,
                   synapse_type=SYN,
                   receptor_type='excitatory')
    sim.Projection(presynaptic_population=population[1:2],
                   postsynaptic_population=population[3:4],
                   connector=CON,
                   synapse_type=SYN,
                   receptor_type='excitatory')

    sim.initialize(population, v=population.get('v_rest'))

    return population


circuit = create_brain()
