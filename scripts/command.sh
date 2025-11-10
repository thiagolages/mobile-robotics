#!/bin/bash

# Start robot control script
$HOME/venv/bin/python $HOME/workspace/src/app.py &

# Start CoppeliaSim
$HOME/workspace/scripts/start_coppeliasim.sh $HOME/workspace/src/assignments/tp3/code/TP3-static.ttt

sleep infinity