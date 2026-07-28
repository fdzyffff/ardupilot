#!/usr/bin/env bash

mkdir -p multi_sitl_test/mavproxy
cd multi_sitl_test/mavproxy || exit 1

mavproxy.py --out 127.0.0.1:14550 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501
