#!/bin/bash

# Get the absolute path of the script's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Add the 'utils' directory to PYTHONPATH
export PYTHONPATH=$DIR:$PYTHONPATH

