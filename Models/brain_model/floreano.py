#-*- coding: utf-8 -*-
"""
This file contains the setup of the neuronal network running the Floreano experiment with neuronal image recognition
"""

__author__ = 'Stefan Walke'

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np
import logging

logger = logging.getLogger(__name__)


# this is the result of 30 generations of evoluttionary learning
dna = np.array([[1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,1,1,1,1,0,0,1,1,0,0,0,1,0],
[0,1,0,1,0,0,0,0,0,1,0,0,1,1,1,1,0,1,1,1,1,0,1,0,1,0,0,1,0],
[0,0,1,1,0,0,0,1,0,0,1,1,1,1,0,0,0,0,1,1,0,1,1,1,0,1,0,1,0],
[0,0,1,1,0,1,0,1,1,1,1,1,1,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0],
[0,1,1,1,1,0,0,1,0,0,1,0,1,0,1,0,0,0,0,0,1,1,0,0,0,1,1,1,1],
[0,0,1,1,0,1,1,0,1,0,0,0,1,1,1,0,0,1,0,1,0,0,1,1,1,0,0,1,1],
[0,0,1,1,0,1,1,1,1,1,1,1,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0,1,1],
[0,0,1,1,1,1,1,0,0,0,0,0,0,1,0,0,1,1,0,0,1,1,0,0,0,1,1,0,0],
[1,0,1,1,0,0,0,1,1,1,1,0,1,0,1,0,1,0,1,0,0,0,1,1,1,1,0,0,1],
[1,1,0,0,0,0,0,1,1,0,1,0,1,0,1,1,0,1,1,0,1,1,0,1,1,0,0,1,0]])


"""
Structure of the dna array: 
-Binary values
-Each row repersents a brain neuron
-First bit: the neurons axons are connected to inhibitory (0) or excitatory (1) synapses
-Next 18 bits: which sensory neurons/receptors are connected (1) or disconnected (0) to/from the neuron (only exitatory synapses)
-Last 10 bits: which brain neurons is the current neuron connected (1) to, self connections allowed
"""

# global variable used to connect the receptors to the correct neurons
receptors = []
for r in range(1,19):
    receptors.append(np.nonzero(dna[:,r])[0])


def create_brain():

    # parameters according to floreano2001, where applicable and else default 
    NEURONPARAMS = {'v_rest': -60.5,
                    'tau_m': 4.0,
                    'tau_refrac': 2.0,
                    'tau_syn_E': 10.0,
                    'tau_syn_I': 10.0,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_thresh': -60.4,
                    'v_reset': -60.5}

    SYNAPSE_PARAMS = {"weight": 1.0,
                      "delay": 2.0}

    population = sim.Population(10, sim.IF_cond_alpha())
    population[0:10].set(**NEURONPARAMS)


    # Connect neurons
    CIRCUIT = population

    SYN = sim.StaticSynapse(**SYNAPSE_PARAMS)

    #iterating over the dna matrix to connect the neurons to each other and determine exitatory/inhibitory feature
    row_counter=0
    for row in dna:
        logger.info(row)
        n = np.array(row)
        r_type = 'excitatory'
        if n[0]==0:
            r_type = 'inhibitory'
        for i in range(19,29):
            if n[i]==1:
                sim.Projection(presynaptic_population=CIRCUIT[row_counter:1+row_counter], postsynaptic_population=CIRCUIT[i-19:i-18], connector=sim.OneToOneConnector(), synapse_type=SYN, receptor_type=r_type)
        
        row_counter+=1

    sim.initialize(population, v=population.get('v_rest'))

    logger.debug("Circuit description: " + str(population.describe()))

    return population


circuit = create_brain()

