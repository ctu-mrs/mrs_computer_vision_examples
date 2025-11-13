#!/bin/bash

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd "$MY_PATH"

# activate the environment
source ./python-env/bin/activate

# install the requirements
python3 -m pip install -r requirements.txt
