current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board fmuv3
./waf copter
# mkdir firmware/fmuv3
cp build/fmuv3/bin/arducopter.apj firmware/$current_datetime\_fmuv3_arducopter.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"