#!/bin/bash
# ./copy-file.sh /home/labinm-jetson/test-records/test-file.txt /home/pqbas/labinm/robot-movil/test-records/

# Check if the required number of parameters is provided
if [ $# -ne 2 ]; then
    echo "No parameters provided. Usage: ./copy-file.sh <file_path> <destination_path>"
    exit 1  # Exit with a non-zero status to indicate an error
fi

# Assign parameters to variables
file_path=$1
dst_path=$2

# Execute the scp command
sshpass -p 'rpgdini100' scp -P 22 "labinm-jetson@192.168.0.10:$file_path" "$dst_path"

# Check the exit status of the sshpass command
if [ $? -ne 0 ]; then
    echo "Error occurred while copying file."
    exit 1  # Exit with a non-zero status to indicate an error
else
    echo "File copied successfully."
fi


