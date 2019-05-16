#!/bin/bash

#  Copyright (C) 2018-2019 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# This script is meant to generate .gcov files from source code built using code coverage compiler flags
# such that .gcno and .gcda files have been generated
# WARNING this script will remove any .gcov files which currently exist under the output directory argument 2. default=./coverage_reports/gcov/
#
# Parameters
# $1 The execution directory which this script will be run from. Generally this should be the catkin workspace with all source code and binaries below this directory
# $2 The output directory which the .gcov files will be written to
#
#

# Helper function to convert the results of find into a bash array
# This runs on the current directory 
#
# $1 An initialized array
# $2 The regex pattern to find this is passed to the -iname command
# $3 Any additional inputs to find
find_as_array() {
	local -n ref_array=$1 # Pass the output array by reference
	# Run the find command and store the results in the array
	while IFS=  read -r -d $'\0'; do
	    ref_array+=("$REPLY")
	done < <(find . -iname "$2" -print0 $3)
}

execution_dir="."
output_dir="./coverage_reports/gcov"

echo "Running gcovr with execution directory $1"
if [ -z "$1" ]; then
	echo "No execution root provided. Executing from ${execution_dir}"
else
	execution_dir=$1;
	cd ${execution_dir} # cd to execution directory
fi

if [ -z "$2" ]; then
	echo "No output directory provided. Output will be in ${output_dir}"
else
	output_dir=$2;
fi

gcovr -k -r . # Run gcovr with -k to ensure generated .gcov files are preserved -r . makes it run in the current directory

echo "Ensuring output directory exists"
mkdir -p ${output_dir}


echo "Deleting old files"
find "${output_dir}" -name "*.gcov" -type f -exec rm -fv {} \;


# Grab resulting gcov files and move them to the ${output_dir} directory
gcov_file_array=()
find_as_array gcov_file_array "*.gcov" "-type f"

echo "Moving new files"

for gcov_file in "${gcov_file_array[@]}"
do
   base_file_name=$(basename ${gcov_file})
   mv ${gcov_file} ${output_dir}/${base_file_name}
done

echo "Files Moved"

exit 0

