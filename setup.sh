#!/bin/bash

sudo apt install python3-colcon-common-extensions
echo "source /usr/share/colcon_argcomplete/hook/colcon_argcomplete.bash" >> ~/.bashrc
source ~/.bashrc

python3 -m pip install --upgrade pip
pip install -e avai_lab[docs]