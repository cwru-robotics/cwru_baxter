#!/bin/bash
#x=1
cd idle
rosrun baxter_examples idle_bash.py
echo idled

cd ..

cd confusion
rosrun baxter_examples confusion.py
echo I was confused

cd ..
cd idle

while :
do
rosrun baxter_examples idle.py
echo idle
done