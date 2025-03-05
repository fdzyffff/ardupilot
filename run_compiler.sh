current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board APzH7
./waf plane
# mkdir firmware/APzH7
cp build/APzH7/bin/arduplane.apj firmware/$current_datetime\_APzH7_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzH7-sim
./waf plane
# mkdir firmware/APzH7
cp build/APzH7/bin/arduplane.apj firmware/$current_datetime\_APzH7-sim_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4
./waf plane
# mkdir firmware/APzF4
cp build/APzF4/bin/arduplane.apj firmware/$current_datetime\_APzF4_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4-sim
./waf plane
# mkdir firmware/APzF4
cp build/APzF4/bin/arduplane.apj firmware/$current_datetime\_APzF4-sim_arduplane.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"
cp -r ArduPlane/param firmware/