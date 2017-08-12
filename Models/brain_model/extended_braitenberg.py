# -*- coding: utf-8 -*-
"""
This file contains the setup of the neuronal network running the Husky experiment with neuronal image recognition
"""
# pragma: no cover

__author__ = 'Lazar Mateev, Georg Hinkel'


from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np
import logging

logger = logging.getLogger(__name__)

def create_brain():
    """
    Initializes PyNN with the neuronal network that has to be simulated

    :returns: The population object
    """

    ##
    ## Set up neurons
    ##

    INPUT_PARAMS = {'a': 4.0,
                    'b': 0.0000805,
                    'delta_T': 2.0,
                    'tau_w': 144.0,
                    'v_spike': 0.0,
                    'cm': .281, # ev. /1000
                    'v_rest': -70.6,
                    'tau_m': 9.3666667,
                    'e_rev_E': 0.0,
                    'e_rev_I': -80.0,
                    'v_reset': -70.6,
                    'v_thresh': -50.4,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 5.,
                    'tau_syn_I': 5.}

    SENSORPARAMS = {'b': 0.0,
                    'tau_w': 10.0,
                    'v_spike': 0.0,
                    'cm': 0.025,
                    'v_rest': -60.5,
                    'tau_m': 10.,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -60.5,
                    'v_thresh': -60.0,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 7.5}

    GO_ON_PARAMS = {'cm': .025,
                    'v_rest': -60.5,
                    'tau_m': 10.,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -61.6,
                    'v_thresh': -60.51,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 7.5}

    INTERMEDIATE_PARAMS = {'a': 4.0,
                           'b': 0.0000805,
                           'delta_T': 2.0,
                           'tau_w': 144.0,
                           'v_spike': 0.0,
                           'cm': .281, # ev. /1000
                           'v_rest': -70.6,
                           'tau_m': 112.4,
                           'e_rev_E': 0.0,
                           'e_rev_I': -80.0,
                           'v_reset': -70.6,
                           'v_thresh': -50.4,
                           'tau_refrac': 10.0,
                           'tau_syn_E': 5.,
                           'tau_syn_I': 5.}


    population = sim.Population(709, sim.EIF_cond_alpha_isfa_ista())

    population[:600].set(**INPUT_PARAMS)
    population[600:605].set(**SENSORPARAMS)
    population[605:606].set(**GO_ON_PARAMS)
    population[606:608].set(**SENSORPARAMS)
    population[608:708].set(**INTERMEDIATE_PARAMS)
    population[708:709].set(**GO_ON_PARAMS)

    ##
    ## Set up synapse types
    ##

    SYNAPSE_PARAMS = {
        "delay": 0.1,
    }

    # Synaptic weights
    weight_red_to_actor = 1.5e-2  # was 1.5e-4
    weight_red_to_go_on = 1000  # or -1.2e-3?
    weight_green_blue_to_actor = 1.0
    weight_go_on_to_right_actor = 1.4e-4  # was 1.4e-4
    weight_eval_red = 2.5e-2
    weight_eval_to_red_sensor = 8.75e-5  # was 1.0e-4, then 8.75e-5
    weight_eval_to_bg_sensor = 5.0  # was 1.5, then 10.0, change it further
    weight_red_to_blue_eval = 1000  # was 5e-8

    synapse_red_to_actor = sim.StaticSynapse(weight=weight_red_to_actor, **SYNAPSE_PARAMS)
    synapse_red_to_go_on = sim.StaticSynapse(weight=weight_red_to_go_on,**SYNAPSE_PARAMS)
    synapse_green_blue_to_actor = sim.StaticSynapse(weight=weight_green_blue_to_actor,
                                                            **SYNAPSE_PARAMS)
    synapse_go_on_to_right_actor = sim.StaticSynapse(weight=weight_go_on_to_right_actor,
                                                            **SYNAPSE_PARAMS)
    synapse_eval_red = sim.StaticSynapse(weight=weight_eval_red, **SYNAPSE_PARAMS)
    synapse_eval_to_red_sensor = sim.StaticSynapse(weight=weight_eval_to_red_sensor,
                                                            **SYNAPSE_PARAMS)
    synapse_eval_to_bg_sensor = sim.StaticSynapse(weight=weight_eval_to_bg_sensor,
                                                            **SYNAPSE_PARAMS)
    synapse_red_to_blue_eval = sim.StaticSynapse(weight=weight_red_to_blue_eval,
                                                            **SYNAPSE_PARAMS)

    ##
    ## Set up projections
    ##

    sim.Projection(population[602:603], population[607:608],
                   connector=sim.AllToAllConnector(),
                   synapse_type=synapse_red_to_actor,
                   receptor_type='excitatory')
    sim.Projection(population[603:604], population[606:607],
                   connector=sim.AllToAllConnector(),
                   synapse_type=synapse_red_to_actor,
                   receptor_type='excitatory')


    sim.Projection(population[600:602], population[605:606],
                   connector=sim.AllToAllConnector(),
                   synapse_type=synapse_red_to_go_on,
                   receptor_type='inhibitory')


    sim.Projection(population[604:605], population[607:608],
                   connector=sim.AllToAllConnector(),
                   synapse_type=synapse_green_blue_to_actor,
                   receptor_type='excitatory')


    sim.Projection(population[605:606], population[607:608],
                   connector=sim.AllToAllConnector(),
                   synapse_type=synapse_go_on_to_right_actor,
                   receptor_type='excitatory')


    # connect the left portion of the detector neurons to the left evaluator and the right portion to the right one
    sim.Projection(population[0:300], population[608:658],
                   connector=sim.FixedNumberPreConnector(6),
                   synapse_type=synapse_eval_red,
                   receptor_type='excitatory')

    sim.Projection(population[300:600], population[658:708],
                   connector=sim.FixedNumberPreConnector(6),
                   synapse_type=synapse_eval_red,
                   receptor_type='excitatory')


    sim.Projection(population[608:658], population[600:601],
                   connector=sim.AllToAllConnector(),
                   synapse_type=synapse_eval_to_red_sensor,
                   receptor_type='excitatory')
    sim.Projection(population[608:658], population[602:603],
                   connector=sim.AllToAllConnector(),
                   synapse_type=synapse_eval_to_red_sensor,
                   receptor_type='excitatory')
    sim.Projection(population[658:708], population[601:602],
                   connector=sim.AllToAllConnector(),
                   synapse_type=synapse_eval_to_red_sensor,
                   receptor_type='excitatory')
    sim.Projection(population[658:708], population[603:604],
                   connector=sim.AllToAllConnector(),
                   synapse_type=synapse_eval_to_red_sensor,
                   receptor_type='excitatory')


    sim.Projection(population[600:602], population[604:605],
                   connector=sim.AllToAllConnector(),
                   synapse_type=synapse_red_to_blue_eval,
                   receptor_type='inhibitory')

    sim.Projection(population[708:709], population[604:605],
                   connector=sim.OneToOneConnector(),
                   synapse_type=synapse_eval_to_bg_sensor,
                   receptor_type='excitatory')  # connect the "blue green evaluator" to the blue green sensor

    sim.initialize(population, v=population.get('v_reset'))

    logger.debug("Circuit description: " + str(population.describe()))

    return population

circuit = create_brain()
sensors = circuit[0:605]
actors = circuit[605:608]
evaluators = circuit[608:709]
