#!/bin/bash

# launch simulation first
./simviz01 &
SIMVIZ_PID=$!

sleep 1

# launch controller
./controller01 &
CONTROLLER_PID=$!

sleep 1

# launch interfaces server
python3 interface/server.py 01-non_redundant_arm.html &
SERVER_PID=$!

# wait for simviz to quit
wait $SIMVIZ_PID

# onnce simviz dies, kill controller & interfaces server
kill $CONTROLLER_PID
for pid in $(ps -ef | grep interface/server.py | awk '{print $2}'); do kill -9 $pid; done
