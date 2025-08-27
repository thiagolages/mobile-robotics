#!/bin/bash

# Start robot control script
/home/thiago/venv/bin/python $HOME/workspace/src/app.py &

# Start CoppeliaSim
/home/thiago/workspace/scripts/start_coppeliasim.sh
