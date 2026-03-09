current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
pre_fix=$"LS-"
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0LDX7
./waf plane
cp build/0LDX7/bin/arduplane.apj firmware/$pre_fix$current_datetime\_0LDX7_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0LDX7-sim
./waf plane
cp build/0LDX7-sim/bin/arduplane.apj firmware/$pre_fix$current_datetime\_0LDX7-sim_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board CUAVv5
./waf plane
cp build/CUAVv5/bin/arduplane.apj firmware/$pre_fix$current_datetime\_CUAVv5_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzH7
./waf plane
cp build/APzH7/bin/arduplane.apj firmware/$pre_fix$current_datetime\_APzH7_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzH7-sim
./waf plane
cp build/APzH7-sim/bin/arduplane.apj firmware/$pre_fix$current_datetime\_APzH7-sim_arduplane.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"