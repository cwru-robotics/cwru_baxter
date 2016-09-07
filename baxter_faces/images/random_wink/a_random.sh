#!/bin/bash
#Executes idle_bash.py or wink_bash.py randomly with a higher probability of executing idle than wink.

while :
do
if (( RANDOM %10 )); then rosrun baxter_examples idle_bash.py && echo idled; else rosrun baxter_examples wink_bash.py && echo winked; fi
done
