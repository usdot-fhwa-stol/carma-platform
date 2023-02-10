#!/bin/sh
showUsage()
{
    echo
    echo "usage: -a remote_address -k key_file -p remote_port -r remote_host_port -u remote_user -t temp_file"
    echo
    exit 1
}
while getopts "p:u:a:t:k:r:" opt; do
    case $opt in
        p)
            REMOTE_PORT="$OPTARG"
            ;;
        u)
            REMOTE_USER="$OPTARG"
            ;;
        a)
            REMOTE_ADDR="$OPTARG"
            ;;
        t)
            TEMP_FILE="$OPTARG"
            ;;
        k)
            KEY_FILE="$OPTARG"
            ;;
        r)
            REMOTE_HOST_PORT="$OPTARG"
            ;;
    esac
done
if [ -z "$TEMP_FILE" ]; then
    TEMP_FILE="/dev/shm/rt_temp.txt"
fi
if [ -z "$REMOTE_PORT" ] || [ -z "$REMOTE_USER" ] || [ -z "$REMOTE_ADDR" ] || [ -z "$KEY_FILE" ]; then
    showUsage
fi

#kill remote process that is running on port $REMOTE_PORT
if ssh -i $KEY_FILE ubuntu@carma-cloud.com "sudo lsof -Pi :$REMOTE_PORT -sTCP:LISTEN" >/dev/null; then
    ssh -i $KEY_FILE ubuntu@carma-cloud.com "sudo lsof -i:$REMOTE_PORT -t | xargs kill " >>/dev/null 2>&1;
fi

# open http tunnel, port-forwarding from  REMOTE_PORT to port 22222 (222222: running in v2xhub)
CMD="/usr/bin/ssh -4 -f -i $KEY_FILE -N -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -p $REMOTE_HOST_PORT -R $REMOTE_PORT:localhost:22222 -T $REMOTE_USER@$REMOTE_ADDR" # -f runs ssh in background after it authenticates, -N creates a tunnel without running remote commands to save resources, -T disables pseudo-tty allocation (each article I read on reverse tunneling used these options
ps -x -o pid,cmd | grep "$CMD" > $TEMP_FILE # write the contents of the ps command to a file, it only prints the pid and the command line command used to create the entry
while read -r line; do # read the lines of the file with ps command
    case "$line" in
        grep) # skip the entry that contains grep
            ;;
        *)
            echo $line # everything else should be a process to kill since the lines in the file were filtered by grep
            PID_TO_KILL=`echo $line | cut -d " " -f 1` # the entries are space delimited, the pid is in position 1
            kill -9 $PID_TO_KILL
            ;;
    esac
done < $TEMP_FILE
rm $TEMP_FILE
exec $CMD # create the new tunnel

