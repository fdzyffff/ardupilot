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
./waf configure --board APzH7
./waf heli
# mkdir firmware/APzH7
cp build/APzH7/bin/arducopter-heli.apj firmware/$current_datetime\_APzH7_arducopter-heli.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4
./waf copter
# mkdir firmware/APzF4
cp build/APzF4/bin/arducopter.apj firmware/$current_datetime\_APzF4_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board MatekH743
./waf copter
# mkdir firmware/MatekH743
cp build/MatekH743/bin/arducopter.apj firmware/$current_datetime\_MatekH743_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzH7-sim
./waf copter
# mkdir firmware/APzH7
cp build/APzH7-sim/bin/arducopter.apj firmware/$current_datetime\_APzH7_sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4-sim
./waf copter
# mkdir firmware/APzF4
cp build/APzF4-sim/bin/arducopter.apj firmware/$current_datetime\_APzF4_sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board MatekH743-sim
./waf copter
# mkdir firmware/MatekH743-sim
cp build/MatekH743-sim/bin/arducopter.apj firmware/$current_datetime\_MatekH743_sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"