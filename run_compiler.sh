current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board 0x7u1.4
./waf copter
# mkdir firmware/0x7u1.4
cp build/0x7u1.4/bin/arducopter.apj firmware/$current_datetime\_0x7u1.4_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board CUAV-X7
./waf copter
# mkdir firmware/0x7u1.4
cp build/CUAV-X7/bin/arducopter.apj firmware/$current_datetime\_CUAV-X7_arducopter.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"