current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board APzF4
./waf copter
# mkdir firmware/APzF4
cp build/APzF4/bin/arducopter.apj firmware/$current_datetime\_APzF4_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board APzF4
./waf rover
# mkdir firmware/APzF4
cp build/APzF4/bin/ardurover.apj firmware/$current_datetime\_APzF4_ardurover.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"