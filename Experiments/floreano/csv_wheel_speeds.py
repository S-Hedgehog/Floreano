@nrp.MapCSVRecorder("recorder", filename="wheel_speeds.csv", headers=["time", "Left_wheel_speed", "Right_wheel_speed"])
@nrp.MapRobotSubscriber("joint_state", Topic('/joint_states', gazebo_msgs.msg.ModelStates))

#global variable for input calculation
@nrp.MapVariable("real_wheel_speed", global_key="real_wheel_speed", initial_value=[0.0,0.0], scope=nrp.GLOBAL)

@nrp.Robot2Neuron()
def csv_wheel_speeds(t, joint_state, recorder, real_wheel_speed):
	"""
    The transfer function which records the turn speed of the robot's wheels during simulation.
    """
    if not isinstance(joint_state.value, type(None)):
    	# add a recorder entry according to the joint state velocity  multiplied by constants that represent the wheels geometry
        recorder.record_entry(t, joint_state.value.velocity[0]*(0.3555/2.0), joint_state.value.velocity[1]*(0.3555/2.0))

        # calculate the values and overwrite the previous wheel speeds values for new input calculations
        lw = joint_state.value.velocity[0]*(0.3555/2.0)
        rw = joint_state.value.velocity[1]*(0.3555/2.0)
        real_wheel_speed.value = [lw,rw]
