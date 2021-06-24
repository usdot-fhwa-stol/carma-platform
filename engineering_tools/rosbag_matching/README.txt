Rosbag Matcher

Matches a given rosbag to a log directory in a given log folder by examining timestamps in the directories and renaming it to match the bag if the difference in timestamps is a minute or less. 

Usage:
source rosbag_match <rosbag> <log folder>

Note:
script might initially take a few seconds to start up. Occasionally, a warning that says
'close failed in file object destructor: sys.excepthook is missing' appears. This can be ignored.

Use the script in a directory with .bag files and a log folder that contains log directories from the same run.

