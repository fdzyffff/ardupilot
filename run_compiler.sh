current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
pre_fix=$"TXHY-"
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board ZDYKfmuPro
./waf copter
cp build/ZDYKfmuPro/bin/arducopter.apj firmware/$pre_fix$current_datetime\_ZDYKfmuPro_arducopter.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board ZDYKfmuPro
./waf plane
cp build/ZDYKfmuPro/bin/arduplane.apj firmware/$pre_fix$current_datetime\_ZDYKfmuPro_arduplane.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"