#!/bin/bash
tests=(
     'cooperativemerge'
     'geometry'
     'guidance'
     'guidance_plugin_api'
     'interfacemgr'
     'lateral_control_driver'
     'message'
     'mock_drivers'
     'platooning'
     'rosutils'
     'route'
     'rsumetering'
     'speedharm'
     'template'
   )
for test in "${tests[@]}"
do
   echo "The current test is $test"
   success=$(./gradlew :$test:test | grep 'BUILD SUCCESSFUL')
   if [[ $success == "BUILD SUCCESSFUL" ]];
   then
      resultString=$'Success \n'
      echo $resultString
      echo ""
      echo ""
   else
      resultString=$'Unsuccessful \n'
      echo $resultString
      echo ""
      echo ""
   fi
done

