#!/bin/bash
#x=1
cd sleeping
rosrun baxter_examples sleeping.py
echo Slept
cd ..
cd waking
rosrun baxter_examples waking.py
echo Woke up
cd ..
cd idle
while :
do
rosrun baxter_examples idle_bash.py
echo Awake
done