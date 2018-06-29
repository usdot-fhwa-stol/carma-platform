#!/bin/bash
result=${PWD##*/}
under="_"  
date=$1
newfolder=$result$under$date
cd ../
mkdir $newfolder
cd $result
echo $newfolder
for folder in */ ; do
	cp ./topics.txt ./$folder
	cp ./only_one.sh ./$folder
	cd ./$folder
	count=`ls -1 *.bag 2>/dev/null | wc -l`
	if [ $count != 0 ]
	then 
		files=( ./*.bag )
		bagfile="${files[0]}"
		bash ./only_one.sh $bagfile
		sleep 1s
		for infolder in */ ; do
			mv $infolder ../../$newfolder		
		done
		sleep 1s
		rm ./topics.txt
		rm ./only_one.sh
		cd ../
	else
		echo "A bag file was not found"
	fi
done
