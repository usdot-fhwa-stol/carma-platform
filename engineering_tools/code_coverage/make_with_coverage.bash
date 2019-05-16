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

# This script is meant to generate .gcov files for code coverage analysis by sonar cloud
# This script depends on collect_gcovr.bash
# This script requires that gcovr be installed
# WARNING this script will remove any .gcov files which currently exist under the output directory argument 2. default=./coverage_reports/gcov/
#
# Required Options
# Either -m or -t or both must be set
#	-m Call make: If set then catkin_make install will be called
# -t Run tests: If set then catkin_make run_tests will be called
# Additional Options
# -e The execution directory which this script will be run from. Generally this should be the catkin workspace with all source code and binaries below this directory
# -o The output directory which the .gcov files will be written to. Relative paths in the output directory will be resolved relative to the execution directory
#
#

usage() { echo "Usage: make_with_coverage.bash -e <execution dir> -o <output dir> -m -t ";}

 

# Default environment variables
execution_dir="."
output_dir="./coverage_reports/gcov"
do_make=false
do_test=false

while getopts e:o:mt option
do
	case "${option}"
	in
		e) execution_dir=${OPTARG};;
		o) output_dir=${OPTARG};;
		m) do_make=true;;
		t) do_test=true;;
		\?) echo "Unknown option: -$OPTARG" >&2; exit 1;;
		:) echo "Missing option argument for -$OPTARG" >&2; exit 1;;
		*) echo "Unimplemented option: -$OPTARG" >&2; exit 1;;

	esac
done

if [ "${do_make}" = false && "${do_test}" = false ]; then
  echo "Error -t or -m must be specified"
  exit 0
fi

execution_dir=$(readlink -f ${execution_dir}) # Get execution directory as absolute path
cd ${execution_dir} # cd to execution directory
echo "Execution Dir: ${execution_dir}"

output_dir=$(readlink -f ${output_dir}); # Get output directory as absolute path
echo "Output Dir: ${output_dir}"


echo "Building and running tests with code coverage"
COVERAGE_FLAGS="-g --coverage -fprofile-arcs -ftest-coverage"

if [ "${do_make}" = true ]; then
  echo "Calling catkin_make"
  catkin_make install -DCMAKE_CXX_FLAGS="${COVERAGE_FLAGS}" -DCMAKE_C_FLAGS="${COVERAGE_FLAGS}" -DCMAKE_BUILD_TYPE="Debug"
fi

if [ "${do_test}" = true ]; then
  echo "Calling catkin_make run_tests"
  catkin_make run_tests -DCMAKE_CXX_FLAGS="${COVERAGE_FLAGS}" -DCMAKE_C_FLAGS="${COVERAGE_FLAGS}" -DCMAKE_BUILD_TYPE="Debug"
fi

bash collect_gcovr.bash "${execution_dir}" "${output_dir}"

echo "Test coverage complete"
