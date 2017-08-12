# -*- coding: utf-8 -*-
"""
This file contains the setup of the neuronal network running the Husky experiment with neuronal image recognition
"""
# pragma: no cover

__author__ = 'Lazar Mateev, Georg Hinkel, Felix Schneider'

import logging
import numpy as np
#import hbp_nrp_cle.brainsim.pynn.simulator as sim
import pyNN.spiNNaker as sim

logger = logging.getLogger(__name__)

def create_brain():
    """
    Initializes PyNN with the neuronal network that has to be simulated
    """
    # sim.setup(timestep=1, min_delay=1, max_delay=20.0, threads=1, debug=True)
    C_m = 25.0  # 25.0, 0.025, stimmen die Einheiten?
    g_L = 2.5  # stimmt das ueberhaupt oder ist g_L was anderes?
    t_m = C_m / g_L

    SENSORPARAMS = {'a': 0.0,
                    'b': 0.0,
                    'delta_T': 0.0,
                    'tau_w': 10.0,
                    'v_spike': 0.0,
                    'cm': C_m,  # 25.0 or 0.025?
                    'v_rest': -60.5,
                    'tau_m': t_m,
                    # 'i_offset': 0.0,			# in der HDF5-Datei nicht vorhanden
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -60.5,
                    'v_thresh': -60.0,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 2.5}

    GO_ON_PARAMS = {'a': 0.0,
                    'b': 0.0,
                    'delta_T': 0.0,
                    'tau_w': 10.0,
                    'v_spike': 0.0,
                    'cm': C_m,
                    'v_rest': -60.5,
                    'tau_m': t_m,
                    # 'i_offset': 0.0,
                    'e_rev_E': 0.0,
                    'e_rev_I': -75.0,
                    'v_reset': -61.6,
                    'v_thresh': -60.51,
                    'tau_refrac': 10.0,
                    'tau_syn_E': 2.5,
                    'tau_syn_I': 2.5}

    formatted_circuit = {
        "x": np.float64(np.array([0, 1, 2, 3, 4, 5, 6, 7])),
        "y": np.float64(np.array([0, 1, 2, 3, 4, 5, 6, 7])),
        "z": np.float64(np.array([0, 1, 2, 3, 4, 5, 6, 7])),
        "layer": np.int16(np.array([1, 1, 1, 1, 1, 2, 3, 3])),
        "mtype": np.array(['SC', 'SC', 'SC', 'SC', 'SC', 'GOC', 'AC', 'AC']),
        "a": np.float64(np.array([SENSORPARAMS.get('a')] * 5 + [GO_ON_PARAMS.get('a')] +
                                 [SENSORPARAMS.get('a')] * 2)),
        "b": np.float64(np.array([SENSORPARAMS.get('b')] * 5 + [GO_ON_PARAMS.get('b')] +
                                 [SENSORPARAMS.get('b')] * 2)),
        "V_th": np.float64(np.array([SENSORPARAMS.get('v_thresh')] * 5 +
                                    [GO_ON_PARAMS.get('v_thresh')] +
                                    [SENSORPARAMS.get('v_thresh')] * 2)),
        "Delta_T": np.float64(np.array([SENSORPARAMS.get('delta_T')] * 5 +
                                       [GO_ON_PARAMS.get('delta_T')] +
                                       [SENSORPARAMS.get('delta_T')] * 2)),
        "C_m": np.float64(
            np.array([SENSORPARAMS.get('cm')] * 5 + [GO_ON_PARAMS.get('cm')] +
                     [SENSORPARAMS.get('cm')] * 2)),
        "g_L": np.float64(np.array([g_L] * 8)),
        "V_reset": np.float64(np.array([SENSORPARAMS.get('v_reset')] * 5 +
                                       [GO_ON_PARAMS.get('v_reset')] +
                                       [SENSORPARAMS.get('v_reset')] * 2)),
        "tau_w": np.float64(np.array([SENSORPARAMS.get('tau_w')] * 5 +
                                     [GO_ON_PARAMS.get('tau_w')] +
                                     [SENSORPARAMS.get('tau_w')] * 2)),
        "t_ref": np.float64(np.array([SENSORPARAMS.get('tau_refrac')] * 5 +
                                     [GO_ON_PARAMS.get('tau_refrac')] +
                                     [SENSORPARAMS.get('tau_refrac')] * 2)),
        "V_peak": np.float64(np.array([SENSORPARAMS.get('v_spike')] * 5 +
                                      [GO_ON_PARAMS.get('v_spike')] +
                                      [SENSORPARAMS.get('v_spike')] * 2)),
        "E_L": np.float64(np.array([SENSORPARAMS.get('v_rest')] * 5 +
                                   [GO_ON_PARAMS.get('v_rest')] +
                                   [SENSORPARAMS.get('v_rest')] * 2)),
        "E_ex": np.float64(np.array([SENSORPARAMS.get('e_rev_E')] * 5 +
                                    [GO_ON_PARAMS.get('e_rev_E')] +
                                    [SENSORPARAMS.get('e_rev_E')] * 2)),
        "E_in": np.float64(np.array([SENSORPARAMS.get('e_rev_I')] * 5 +
                                    [GO_ON_PARAMS.get('e_rev_I')] +
                                    [SENSORPARAMS.get('e_rev_I')] * 2)),
        "tau_syn_E": np.float64(np.array([SENSORPARAMS.get('tau_syn_E')] * 5 +
                                         [GO_ON_PARAMS.get('tau_syn_E')] +
                                         [SENSORPARAMS.get('tau_syn_E')] * 2)),
        "tau_syn_I": np.float64(np.array([SENSORPARAMS.get('tau_syn_I')] * 5 +
                                         [GO_ON_PARAMS.get('tau_syn_I')] +
                                         [SENSORPARAMS.get('tau_syn_I')] * 2)),
        "excitatory": np.array([90, 90, 110, 110, 110, 110, 110, 110])
    }
    params = {
        "v_thresh": formatted_circuit["V_th"],
        "cm": formatted_circuit["C_m"] * 1e-3,
        "tau_m": formatted_circuit["C_m"] / formatted_circuit["g_L"],
        "v_reset": formatted_circuit["V_reset"],
        "tau_refrac": formatted_circuit["t_ref"],
        "v_rest": formatted_circuit["E_L"],
        "e_rev_E": formatted_circuit["E_ex"],
        "e_rev_I": formatted_circuit["E_in"],
        "tau_syn_E": formatted_circuit["tau_syn_E"],
        "tau_syn_I": formatted_circuit["tau_syn_I"]
    }

    # SpiNNaker does not support Population subindexing, so we need to set up populations differently.
    # SpiNNaker also does not support IF_cond_alpha neurons. So we use IF_cond_exp instead. Should be the same, right?
    cells = [sim.Population(1, sim.IF_cond_exp, dict([(key, value[i]) for key, value in params.items()])) for i in range(8)]
    # SpiNNaker does not support fast plasticity synapse dynamics.
    # params = {'U': 1.0, 'tau_rec': 0.0, 'tau_facil': 0.0}
    # syndynamics = sim.SynapseDynamics(fast=sim.TsodyksMarkramMechanism(**params))

    # Synaptic weights
    SCALE = 2.5
    WEIGHT_RED_TO_ACTOR = 1.5e-4 * SCALE
    WEIGHT_RED_TO_GO_ON = 1.2e-3 * SCALE
    WEIGHT_GREEN_BLUE_TO_ACTOR = 1.05e-4 * SCALE
    WEIGHT_GO_ON_TO_RIGHT_ACTOR = 1.4e-4 * SCALE
    DELAY = 0.1

    # Connect neurons
    CONN = sim.AllToAllConnector(weights=abs(WEIGHT_RED_TO_ACTOR),
                                 delays=DELAY)
    sim.Projection(cells[2], cells[7], CONN, target='excitatory')
    sim.Projection(cells[3], cells[6], CONN, target='excitatory')

    CONN = sim.AllToAllConnector(weights=abs(WEIGHT_RED_TO_GO_ON),
                                 delays=DELAY)
    sim.Projection(cells[0], cells[5], CONN, target='inhibitory')
    sim.Projection(cells[1], cells[5], CONN, target='inhibitory')

    CONN = sim.AllToAllConnector(weights=abs(WEIGHT_GREEN_BLUE_TO_ACTOR),
                                 delays=DELAY)
    sim.Projection(cells[4], cells[7], CONN, target='excitatory')

    CONN = sim.AllToAllConnector(weights=abs(WEIGHT_GO_ON_TO_RIGHT_ACTOR),
                                 delays=DELAY)
    sim.Projection(cells[5], cells[7], CONN, target='excitatory')

    for pop in cells:
        sim.initialize(pop, 'v', pop.get('v_rest'))

    return cells

circuit = create_brain()

