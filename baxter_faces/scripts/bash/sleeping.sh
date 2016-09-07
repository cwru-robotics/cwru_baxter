#!/bin/bash
#x=1
cd idle
rosrun baxter_examples idle_bash.py
echo idled

cd ..

cd sleeping
rosrun baxter_examples falling_asleep.py
echo Falling asleep

while :
do
rosrun baxter_examples sleeping.py
echo Asleep
done