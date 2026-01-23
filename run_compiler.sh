current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0LDX7
./waf plane
cp build/0LDX7/bin/arduplane.apj firmware/$current_datetime\_0LDX7_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board 0LDX7-sim
./waf plane
cp build/0LDX7-sim/bin/arduplane.apj firmware/$current_datetime\_0LDX7-sim_arduplane.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"
./waf configure --board CUAVv5Nano
./waf plane
cp build/CUAVv5Nano/bin/arduplane.apj firmware/$current_datetime\_CUAVv5Nano_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"