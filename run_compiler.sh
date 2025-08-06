current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0X7u
./waf copter
# mkdir firmware/0X7u
cp build/0X7u/bin/arducopter.apj firmware/$current_datetime\_0X7u_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0X7u-sim
./waf copter
# mkdir firmware/0X7u
cp build/0X7u-sim/bin/arducopter.apj firmware/$current_datetime\_0X7u_sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"