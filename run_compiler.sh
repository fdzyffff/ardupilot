current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board ZDYKfmuPro
./waf plane
# mkdir firmware/ZDYKfmuPro
cp build/ZDYKfmuPro/bin/arduplane.apj firmware/$current_datetime\_ZDYKfmuPro_arduplane.apj
echo "~~~~~~~~~~~~~~~ next ~~~~~~~~~~~~~~~"
./waf configure --board ZDYKfmuPro-sim
./waf plane
# mkdir firmware/ZDYKfmuPro
cp build/ZDYKfmuPro/bin/arduplane.apj firmware/$current_datetime\_ZDYKfmuPro-sim_arduplane.apj
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"
