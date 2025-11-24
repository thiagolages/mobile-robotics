#!/bin/bash

# Start robot control script
$HOME/venv/bin/python $HOME/workspace/src/app.py &

# Start CoppeliaSim
$HOME/workspace/scripts/start_coppeliasim.sh $HOME/workspace/src/assignments/final/FinalProject.ttt

sleep infinity