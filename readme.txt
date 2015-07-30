Before running the script, specify the mocap poses topic's name in the parameter ROCKIN_MOCAP_TOPIC;
The topic must be of type geometry_msgs/PoseStamped (not Pose2D)

Every output is written in the console and to a file named "rtb_result_for_{robot_bag.bag}.txt" (appending mode is used, the file is not overwritten if the script is run multiple times on the same robot bag)

Usage:
rockin_trajectory_benchmark.py teamname rockin_logger_bag_file.bag rockin_mocap_bags_directory
	teamname:						the name that was set in the configuration of rockin_logger, should be the same as the one in the bag's name
	rockin_logger_bag_file.bag:		the bag produced by rockin_logger
	rockin_mocap_bags_directory:	the directory in which are found all the bags produced by rockin_mocap while rockin_logger was being executed

=========================================

It is possible to specify every mocap bag in the arguments instead of the rockin_mocap_bags_directory.
To do so, comment the code where it says "auto-selection of mocap bags to be opened" and uncomment "without mocap bags auto-selection"
It may be usefull to disable the auto-selection if there are a lot of mocap bags to select and the selection takes too much time.

In this case the command line arguments are
    rockin_trajectory_benchmark.py teamname rockin_logger_bag_file.bag rockin_mocap_bag_1 [rockin_mocap_bag_2]...

=========================================

WARNINGs and ERRORs:

	· [ERROR] mocap bags are missing:
Most probably a mocap bag wasn't found; This error occurs when the robot bag and the mocap bag(s) don't completely overlap (ie when the last robot pose was recorded after the end of all the mocap bags).

	· [WARNING] poses out of order:
This warning means that either the mocap poses in the mocap bag are not (chronologically) ordered or that different mocap bags were concatenated badly (ie not in the right order) or even that two selected mocap bags overlap (so the timestamp of the last pose of bag1 is later than the timestamp of the first pose of bag2).

	· [WARNING] consecutive mocap poses very distant in time:
This means that there is a "hole" in the mocap bag and that there aren't available mocap poses for an extended period, this might compromise the correctness of the result.

    · [WARNING] consecutive mocap poses very distant in time, used to calculate the error:
As the previous warning, but this also mean that the couple of mocap poses has been used to calculate the error, so the robot pose and the mocap pose(s) are very distant in time too.
Most probably the error calculated with these particular poses is very imprecise (although the effect on the final result might be negligible).

	· [WARNING] tracking was lost:
When calculating the trajectory error, robot poses taken while the tracking was lost are ignored.

	· [ERROR] mocap poses mp1, mp2 and the robot pose rp with which the error is calculated, weren't such that mp1.t <= rp.t <= mp2.t:
This shouldn't even happen.

=========================================

Note on the programme's performances:
To open and go through
· 2 mocap bags of 1 hour, ~150 MiB each
· robot bag of 35 minutes, ~650 MiB
the process takes ~1.7 GiB of RAM (in 2-3 minutes)

