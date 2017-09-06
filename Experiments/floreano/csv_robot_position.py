@nrp.MapCSVRecorder("recorder", filename="robot_position.csv", headers=["x", "y", "z"])
@nrp.MapRobotSubscriber("position", Topic('/gazebo/model_states', gazebo_msgs.msg.ModelStates))

@nrp.MapVariable("last_position", initial_value=[0.0, 0.0])
@nrp.MapVariable("travel_distance", global_key="dist", initial_value=0.0, scope=nrp.GLOBAL)

@nrp.Robot2Neuron()
def csv_robot_position(t, last_position, travel_distance, position, recorder):

	pos_x = position.value.pose[-1].position.x
	pos_y = position.value.pose[-1].position.y
	pos_z = position.value.pose[-1].position.z
	last = last_position.value
	recorder.record_entry(pos_x, pos_y, pos_z)
	travel_distance.value = travel_distance.value + np.sqrt(np.square(pos_x-last[0]) + np.square(pos_y-last[1]))
	last_position.value = [pos_x, pos_y]
	#clientLogger.info(travel_distance.value)

