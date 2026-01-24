current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board APzH7
./waf copter
# mkdir firmware/APzH7
cp build/APzH7/bin/arducopter.apj firmware/$current_datetime\_APzH7_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board CUAVv5-sim
./waf copter
# mkdir firmware/CUAVv5-sim
cp build/CUAVv5-sim/bin/arducopter.apj firmware/$current_datetime\_CUAVv5-sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"
