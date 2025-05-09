current_datetime=$(date +'%Y%m%d_%H%M%S')
echo $current_datetime
mkdir firmware
rm -fr firmware/*
echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board KakuteH7
./waf copter
# mkdir firmware/KakuteH7
cp build/KakuteH7/bin/arducopter.apj firmware/$current_datetime\_KakuteH7_arducopter.apj 
echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"