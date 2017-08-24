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

l = np.random.randint(2,size=(10,29))
"""
Structure of the dna array: 
-Binary values
-Each row repersents a brain neuron
-First bit: the neurons axons are connected to inhibitory (0) or excitatory (1) synapses
-Next 18 bits: which sensory neurons are connected (1) or disconnected (0) to/from the neuron (only exitatory synapses)
-Last 10 bits: which brain neurons is the current neuron connected (1) to, self connectionbs allowed
"""

def create_brain(dna = l):
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

    SYNAPSE_PARAMS = {"weight": 1,
                      "delay": 2.0,
                      'U': 1.0,
                      'tau_rec': 1.0,
                      'tau_facil': 1.0}

    population = sim.Population(28, sim.IF_cond_alpha())
    population[0:18].set(**SENSORPARAMS)
    population[18:28].set(**BRAINPARAMS)

    # Connect neurons
    CIRCUIT = population

    SYN = sim.TsodyksMarkramSynapse(**SYNAPSE_PARAMS)

    row_counter=0
    for row in dna:
    	logger.info(row)
        n = np.array(row)
        r_type = 'excitatory'
        for i in range(1,19):
            if n[i]==1:
            	logger.info(str(i-1)+' '+str(18+row_counter)+' '+r_type)
                sim.Projection(presynaptic_population=CIRCUIT[i-1:i], postsynaptic_population=CIRCUIT[18+row_counter:19+row_counter], connector=sim.OneToOneConnector(), synapse_type=SYN, receptor_type=r_type)
        if n[0]==0:
            r_type = 'inhibitory'
        for i in range(19,29):
            if n[i]==1:
            	logger.info(str(18+row_counter)+' '+str(i-1)+' '+r_type)

                temp = 2*np.random.random()

                SYNAPSE_PARAMS_2 = {"weight": temp,
                      "delay": 2.0,
                      'U': 1.0,
                      'tau_rec': 1.0,
                      'tau_facil': 1.0}

                SYN2 = sim.TsodyksMarkramSynapse(**SYNAPSE_PARAMS_2)
                
                sim.Projection(presynaptic_population=CIRCUIT[18+row_counter:19+row_counter], postsynaptic_population=CIRCUIT[i-1:i], connector=sim.OneToOneConnector(), synapse_type=SYN2, receptor_type=r_type)
        
        row_counter+=1

    sim.initialize(population, v=population.get('v_rest'))

    logger.debug("Circuit description: " + str(population.describe()))

    return population


circuit = create_brain()
