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
./waf configure --board Pixhawk6X
./waf copter
# mkdir firmware/Pixhawk6X
cp build/Pixhawk6X/bin/arducopter.apj firmware/$current_datetime\_Pixhawk6X_arducopter.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"
