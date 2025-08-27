#!/bin/bash

# Set the correct HOME directory for the thiago user
export HOME=/home/thiago

# Source the bashrc to get the virtual environment setup
source $HOME/.bashrc

# Execute the command passed to the entrypoint
exec "$@"