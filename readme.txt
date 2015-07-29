
Specify the mocap poses topic's name in the parameter ROCKIN_MOCAP_TOPIC in the script file

Usage:
rockin_trajectory_benchmark.py teamname rockin_logger_bag_file.bag rockin_mocap_bags_directory
	teamname:						the name that was set in the configuration of rockin_logger, should be the same as the one in the bag's name
	rockin_logger_bag_file.bag:		the bag produced by rockin_logger
	rockin_mocap_bags_directory:	the directory in which are found all the bags produced by rockin_mocap while rockin_logger was being executed

It is possible to specify every mocap bag in the arguments instead of the rockin_mocap_bags_directory;
comment the code where it says "auto-selection of mocap bags to be opened" and uncomment "without mocap bags auto-selection"

Note on the programme's performances:
To open and go through
· 2 mocap bags of 1 hour, ~150 MiB each
· robot bag of 35 minutes, ~650 MiB
the process takes ~1.7 GiB of RAM (in 2-3 minutes)

