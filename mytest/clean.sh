echo "----cleaning all previous sitl----"
ps -ef|grep sitl|grep -v grep|awk '{print $2}' |xargs kill -9
#rm -r swarm