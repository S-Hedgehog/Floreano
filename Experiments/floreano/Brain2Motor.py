# Imported Python Transfer Function
"""
This module contains the transfer function which is responsible for determining the linear twist
component of the husky's movement based on the left and right wheel neuron
"""
import hbp_nrp_cle.tf_framework as nrp
from hbp_nrp_cle.robotsim.RobotInterface import Topic
import geometry_msgs.msg
@nrp.MapSpikeSink("left_wheel_forward_neuron", nrp.brain.actors[0], nrp.population_rate)
@nrp.MapSpikeSink("left_wheel_back_neuron", nrp.brain.actors[1], nrp.population_rate)
@nrp.MapSpikeSink("right_wheel_forward_neuron", nrp.brain.actors[2], nrp.population_rate)
@nrp.MapSpikeSink("right_wheel_back_neuron", nrp.brain.actors[3], nrp.population_rate)
@nrp.Neuron2Robot(Topic('/husky/cmd_vel', geometry_msgs.msg.Twist))
def Brain2Motor(t, left_wheel_forward_neuron, left_wheel_back_neuron, right_wheel_forward_neuron, right_wheel_back_neuron):
    """
    The transfer function which calculates the linear twist of the husky robot based on the
    voltage of left and right wheel neurons.
    :param t: the current simulation time
    :param left_wheel_forward_neuron: the left wheel forward neuron device
    :param left_wheel_back_neuron: the left wheel back neuron device
    :param right_wheel_forward_neuron: the right wheel forward neuron device
    :param right_wheel_back_neuron: the right wheel back neuron device
    :return: a geometry_msgs/Twist message setting the linear twist fo the husky robot movement.
    """
    left_wheel = 0.08 * (left_wheel_forward_neuron.rate - left_wheel_back_neuron.rate)
    right_wheel = 0.08 * (right_wheel_forward_neuron.rate - right_wheel_back_neuron.rate)
    clientLogger.info(left_wheel, right_wheel)
    linear=geometry_msgs.msg.Vector3(x=(left_wheel + right_wheel)/2.0, y=0.0, z=0.0)
    angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=(right_wheel - left_wheel)/0.5709)
    return geometry_msgs.msg.Twist(linear, angular)
