##Bag Processor

Automated Script to process bag files

	Some notes: 

	This script, process_bags.bash, can be run within the folder of the vehicle. This script depends on process_one_bag.bash, which depends on topics.txt, meaning all three must be in the same directory. Please make sure that there exist bag files within each subdirectory. This means that the bag file must be within its corresponding subdirectory. 

	To omit certain topics from being processed, you can comment it out by putting # before the line in topics.txt.
	
	For example:	

		#/saxton_cav/drivers/srx_can/can/brake_position > brake_position.csv
		/saxton_cav/drivers/srx_can/can/speed > can_speed.csv
		/saxton_cav/drivers/srx_controller/control/cmd_speed > cmd_speed.csv
		#/saxton_cav/drivers/srx_objects/f_lrr/sensor/objects > front_radar.csv
		#/saxton_cav/guidance/outgoing_mobility_operation > outgoing_mobility_operation.csv
		/saxton_cav/guidance/outgoing_mobility_path > outgoing_mobility_path.csv
		/saxton_cav/guidance/outgoing_mobility_request > outgoing_mobility_request.csv
		#/saxton_cav/guidance/outgoing_mobility_response > outgoing_mobility_response.csv
		#/saxton_cav/guidance/plugins/controlling_plugins > controlling_plugin.csv
		#/saxton_cav/message/incoming_bsm > incoming_bsm.csv
		#/saxton_cav/message/incoming_mobility_operation > incoming_mobility_operation.csv
		#/saxton_cav/message/incoming_mobility_request > incoming_mobility_request.csv
		#/saxton_cav/message/incoming_mobility_response > incoming_mobility_response.csv
		#/saxton_cav/message/incoming_mobility_path > incoming_mobility_path.csv
		#/saxton_cav/sensor_fusion/filtered/heading > heading.csv
		#/saxton_cav/sensor_fusion/filtered/velocity > velocity.csv
		#/saxton_cav/sensor_fusion/filtered/nav_sat_fix > nav_sat_fix.csv
		#/saxton_cav/sensor_fusion/filtered/acceleration > acceleration.csv


	Would only process can_speed, cmd_speed, outgoing_mobility_path, and outgoing_mobility_request as topics

		
	To call the script:

		bash ./process_bags.bash <date_of_test>
	
		For Example:
			If the date that the test was conducted is June 27, 2018, then it should be denoted as 20180627
			

		The processed files will be placed in a newly created directory and will be denoted VEHICLE_DATEOFTEST
		
		So the directory will look something like this if process_bags.bash was run inside the white directory
		
		TO26_I95_Logs_6_27_18
		--Black
		--Green
		--Grey
		--Silver
		--White
		--White_20180627 <-- resulting directory
		----process_bags.bash
		----process_one_bag
		----topics.txt
		----20180627-071715
		------_2018-06-27-07-17-16.bag
		----20180627-100653
		------_2018-06-27-10-06-55.bag
		----20180627-104003
		------_2018-06-27-10-40-05.bag
		----20180627-104809
		------_2018-06-27-10-48-10.bag
		----20180627-111624
		------_2018-06-27-11-16-25.bag
		----20180627-123800
		------_2018-06-27-12-38-01.bag
		----20180627-132041
		------_2018-06-27-13-20-42.bag
		




