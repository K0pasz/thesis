#!/bin/bash

KEYFRAMES_PATH=$1

python3 post_process_images.py --directory $KEYFRAMES_PATH

ns-process-data images --data $KEYFRAMES_PATH --output-dir gsplat_input/

ns-train splatfacto-big --data gsplat_input/ --viewer.quit-on-train-completion True

# The ply export can only be done manually because nerfstudio puts the output in a folder with name of a timestamp
# and this cannot be deactivated
