current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
pre_fix=$"ER-"
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0N7
./waf copter
cp build/0N7/bin/arducopter.apj firmware/$pre_fix$current_datetime\_0N7_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0N7-sim
./waf copter
cp build/0N7-sim/bin/arducopter.apj firmware/$pre_fix$current_datetime\_0N7-sim_arducopter.apj
# echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
# ./waf configure --board APzH7
# ./waf copter
# cp build/APzH7/bin/arducopter.apj firmware/$pre_fix$current_datetime\_APzH7_arducopter.apj
# echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
# ./waf configure --board APzH7-sim
# ./waf copter
# cp build/APzH7-sim/bin/arducopter.apj firmware/$pre_fix$current_datetime\_APzH7-sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"