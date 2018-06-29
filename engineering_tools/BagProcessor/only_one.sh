#!/bin/bash
killall roscore || echo "roscore was not running."
killall rosmaster || echo "rosmaster was not running."
bagfile=$1
folder=$(basename "$bagfile" .bag);
command=""
file=topics.txt
while read -r line; do
    [[ "$line" =~ ^#.*$ ]] && continue
    command=$command${line}" "
done < "$file"
command=${command%?}""
command=${command%?}""
command=${command%?}";"
mkdir $folder
mv ./$1 ./$folder
mv route.txt ./$folder
cd $folder
roscore &
sleep 3s
xterm -e "$command" &
sleep 10s
rosbag play -r 10 $bagfile
sleep 5s
killall rostopic
if [[ -s output/nav_sat_fix.csv ]]
then
    echo "nav_sat_fix did not work"
    folder=$(basename "$1" .bag)
    cat route.txt | grep Downtrack: > position.csv;
    mv position.csv $folder
else
    echo "nav_sat_fix worked!"
fi
killall roscore
killall rosmaster
mv ./$1 ../
mv route.txt ../
