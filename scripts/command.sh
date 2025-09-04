#!/bin/bash

# Start robot control script
$HOME/venv/bin/python $HOME/workspace/src/tp1.py &

# Start CoppeliaSim
$HOME/workspace/scripts/start_coppeliasim.sh $HOME/workspace/src/scenes/TP1.ttt

sleep infinity