##Bag Processor

Automated Script to process bag files

	Some notes: 

	A new script can be run within the folder of the vehicle. This script depends on only_one.sh, which depends on topics.txt, meaning all three must be in the same directory. Please make sure that there exist bag files within each subdirectory. This means that the bag file must be within its corresponding subdirectory. 

	To omit certain topics from being processed, you can comment it out by putting # before the line in topics.txt.
	
	For example:	

		#rostopic echo -p /saxton_cav/drivers/srx_can/can/brake_position > brake_position.csv &
		rostopic echo -p /saxton_cav/drivers/srx_can/can/speed > can_speed.csv &
		rostopic echo -p /saxton_cav/drivers/srx_controller/control/cmd_speed > cmd_speed.csv &
		#rostopic echo -p /saxton_cav/drivers/srx_objects/f_lrr/sensor/objects > front_radar.csv &
		#rostopic echo -p /saxton_cav/guidance/outgoing_mobility_operation > outgoing_mobility_operation.csv &
		rostopic echo -p /saxton_cav/guidance/outgoing_mobility_path > outgoing_mobility_path.csv &
		rostopic echo -p /saxton_cav/guidance/outgoing_mobility_request > outgoing_mobility_request.csv &
		#rostopic echo -p /saxton_cav/guidance/outgoing_mobility_response > outgoing_mobility_response.csv &
		#rostopic echo -p /saxton_cav/guidance/plugins/controlling_plugins > controlling_plugin.csv &
		#rostopic echo -p /saxton_cav/message/incoming_bsm > incoming_bsm.csv &
		#rostopic echo -p /saxton_cav/message/incoming_mobility_operation > incoming_mobility_operation.csv &
		#rostopic echo -p /saxton_cav/message/incoming_mobility_request > incoming_mobility_request.csv &
		#rostopic echo -p /saxton_cav/message/incoming_mobility_response > incoming_mobility_response.csv &
		#rostopic echo -p /saxton_cav/message/incoming_mobility_path > incoming_mobility_path.csv &
		#rostopic echo -p /saxton_cav/sensor_fusion/filtered/heading > heading.csv &
		#rostopic echo -p /saxton_cav/sensor_fusion/filtered/velocity > velocity.csv &
		#rostopic echo -p /saxton_cav/sensor_fusion/filtered/nav_sat_fix > nav_sat_fix.csv &
		#rostopic echo -p /saxton_cav/sensor_fusion/filtered/acceleration > acceleration.csv &


	Would only process can_speed, cmd_speed, outgoing_mobility_path, and outgoing_mobility_request as topics

		
	To call the script:

		bash ./all_mine.sh <date_of_test>
	
		For Example:
			If the date that the test was conducted is June 29, 2018, then it should be denoted as 20180629
			

		The processed files will be placed in a newly created directory and will be denoted VEHICLE_DATEOFTEST
		
		So the directory will look something like this if all_mine.sh was run inside the white directory
		
		Black
		Green
		Grey
		Silver
		White
		White_20180629 <-- newly created directory.
		




