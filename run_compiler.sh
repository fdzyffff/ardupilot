current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board APzF400
./waf copter
# mkdir firmware/APzF400
cp build/APzF400/bin/arducopter.apj firmware/$current_datetime\_APzF400_arducopter.apj
# echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APz400-H7
./waf copter
# mkdir firmware/APz400
cp build/APz400-H7/bin/arducopter.apj firmware/$current_datetime\_APz400-H7_arducopter.apj
# echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
# ./waf configure --board APzF4
# ./waf copter
# # mkdir firmware/APzF4
# cp build/APzF4/bin/arducopter.apj firmware/$current_datetime\_APzF4_arducopter.apj
# echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
# ./waf configure --board APzF4
# ./waf heli
# # mkdir firmware/APzF4
# cp build/APzF4/bin/arducopter-heli.apj firmware/$current_datetime\_APzF4_arducopter-heli.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"