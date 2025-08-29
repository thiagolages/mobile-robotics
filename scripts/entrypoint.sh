#!/bin/bash

# Source the bashrc to get the virtual environment setup
source $HOME/.bashrc

# Execute the command passed to the entrypoint
exec "$@"