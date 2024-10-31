#!/bin/bash
#./copy-directory.sh /home/labinm-jetson/test-records/ /home/pqbas/labinm/robot-movil/

# Check if the required number of parameters is provided
if [ $# -ne 2 ]; then
    echo "No parameters provided. Usage: ./copy-directory.sh <source_path> <destination_path>"
    exit 1  # Exit with a non-zero status to indicate an error
fi

# Assign parameters to variables
src_path=$1
dst_path=$2

# Execute the scp command
sshpass -p 'rpgdini100' scp -r -P 22 "labinm-jetson@192.168.0.10:$src_path" "$dst_path"

# Check the exit status of the sshpass command
if [ $? -ne 0 ]; then
    echo "Error occurred while copying files."
    exit 1  # Exit with a non-zero status to indicate an error
else
    echo "Files copied successfully."
fi

# ./copy-directory.sh ~/test-records/ ~/labinm/robot-movil/
# sshpass -p rpgdini100 scp -r -P 22 labinm-jetson@192.168.0.10:~/test-records/ ~/labinm/robot-movil/

