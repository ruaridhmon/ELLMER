#!/bin/bash
mkdir -p $1csv
ROSBAGAMOUNT=$(find $1 -type f -name "*.bag" | wc -l)
ROSBAGCOUNT=0
printf 'Converting .bag files to .csv ...\n'
for bag_file in $1*.bag ; do
for topic in `rostopic list -b $bag_file` ; do
    rostopic echo -p -b $bag_file $topic > ${bag_file//.bag/}_${topic//\//_}.csv 
done
ROSBAGCOUNT=$[$ROSBAGCOUNT +1]
PROGRESS=$[100 * $ROSBAGCOUNT / $ROSBAGAMOUNT]
echo -ne $PROGRESS '% \r'
done
mv $1*.csv $1csv
echo $PROGRESS '% Done!'

