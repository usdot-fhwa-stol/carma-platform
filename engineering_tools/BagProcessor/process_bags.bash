#!/bin/bash
result=${PWD##*/}
under="_"  
date=$1
filter=${2:-0}
rosplay=${3:-5}
echo $filter
newfolder=$result$under$date
cd ../
mkdir $newfolder
cd $result
echo $newfolder
for folder in */ ; do
	cp ./topics.txt ./$folder
	cp ./process_one_bag.bash ./$folder
	cd ./$folder
	count=`ls -1 *.bag 2>/dev/null | wc -l`
	if [ $count != 0 ]
	then 
		files=( ./*.bag )
		bagfile="${files[0]}"
		bash ./process_one_bag.bash $bagfile $filter $rosplay
		sleep 1s
		for infolder in */ ; do
			mv $infolder/ ../../$newfolder/		
		done
		sleep 1s
		rm ./topics.txt
		rm ./process_one_bag.bash
		cd ../
	else
		echo "A bag file was not found"
		sleep 1s
		rm ./topics.txt
		rm ./process_one_bag.bash
		cd ../
	fi
done
