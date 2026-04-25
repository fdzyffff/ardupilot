prefix=$"SANQI"
current_datetime=$prefix"_"$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0LDX7
./waf copter
# mkdir firmware/0LDX7
cp build/0LDX7/bin/arducopter.apj firmware/$current_datetime\_0LDX7_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0LDX7-sim
./waf copter
# mkdir firmware/0x7u1.4
cp build/0LDX7-sim/bin/arducopter.apj firmware/$current_datetime\_0LDX7-sim_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzH7
./waf copter
# mkdir firmware/0x7u1.4
cp build/APzH7/bin/arducopter.apj firmware/$current_datetime\_APzH7_arducopter.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"