current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0x7u1.4
./waf plane
cp build/0x7u1.4/bin/arduplane.apj firmware/$current_datetime\_0x7u1.4_arduplane.apj
# echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
# ./waf configure --board 0x7u1.4-sim
# ./waf plane
# cp build/0x7u1.4-sim/bin/arduplane.apj firmware/$current_datetime\_0x7u1.4-sim_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzH7
./waf plane
cp build/APzH7/bin/arduplane.apj firmware/$current_datetime\_APzH7_arduplane.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"