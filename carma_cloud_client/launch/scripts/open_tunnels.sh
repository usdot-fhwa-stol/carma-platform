#!/bin/sh
showUsage()
{
    echo
    echo "usage: -a remote_address -k key_file  -r host_port -u remote_user -t temp_file -p remote_port"
    echo
    exit 1
}
while getopts "p:u:a:t:k:r:" opt; do
    case $opt in
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
            HOST_PORT="$OPTARG"
            ;;
        p)
            REMOTE_PORT="$OPTARG"
            ;;
    esac
done

if [ $(id -u) -ne 0 ];then
    echo "Please run as root "
    exit
fi

if [ -z "$TEMP_FILE" ]; then
    TEMP_FILE="/dev/shm/rt_temp.txt"
fi


# open http tunnel, port-forwarding from HOST_PORT to port 8080 (8080: running on carma cloud)
if  [ -z "$REMOTE_USER" ] || [ -z "$REMOTE_ADDR" ] || [ -z "$KEY_FILE" ]; then
    showUsage
fi

# kill local process that is running on port HOST_PORT
if sudo lsof -t -i:$HOST_PORT >/dev/null; then
    sudo kill -9 $(sudo lsof -t -i:$HOST_PORT)
    echo "Closed existing host port $HOST_PORT"
fi


# Open forward tunnel: This port (33333) is forwarded to remote host (carma-cloud) and port: 8080 
echo "Open forward tunnel..."

CMD="/usr/bin/ssh -4 -f -i $KEY_FILE -L $HOST_PORT:localhost:8080 -N -o StrictHostKeyChecking=no -o ServerAliveInterval=30 -o ServerAliveCountMax=10 -o UserKnownHostsFile=/dev/null -o LogLevel=quiet $REMOTE_USER@$REMOTE_ADDR" # -f runs ssh in background after it authenticates, -N creates a tunnel without running remote commands to save resources, -T disables pseudo-tty allocation 
ps -x -o pid,cmd | grep "$CMD" > $TEMP_FILE # write the contents of the ps command to a file, it only prints the pid and the command line command used to create the entry
while read -r line; do # read the lines of the file with ps command
    case "$line" in
        grep) # skip the entry that contains grep
            ;;
        *)
            echo $line # everything else should be a process to kill since the lines in the file were filtered by grep
            PID_TO_KILL=`echo $line | cut -d " " -f 1` # the entries are space delimited, the pid is in position 1
            output=$(ps -p "$PID_TO_KILL")
            if [ "$?" -eq 0 ]; then
                kill -9 $PID_TO_KILL
            fi
            ;;
    esac
done < $TEMP_FILE
rm $TEMP_FILE
exec $CMD &> /dev/null  # create the new tunnel

if [ "$?" -eq 0 ]; then  
     echo "Forward tunnel is successfully opened!" 
fi

if sudo lsof -t -i:$HOST_PORT >/dev/null; then
    echo "Forward tunnel is successfully opened!"  
fi




#kill remote process that is running on port $REMOTE_PORT
if ssh -i  $KEY_FILE  $REMOTE_USER@$REMOTE_ADDR "sudo lsof -Pi:$REMOTE_PORT -sTCP:LISTEN" >/dev/null; then
    ssh -i $KEY_FILE  $REMOTE_USER@$REMOTE_ADDR "sudo lsof -i:$REMOTE_PORT -t  | xargs kill " >>/dev/null 2>&1
    echo "Closed exiting remote port $REMOTE_PORT"
fi

# open reverse tunnel:  carma-cloud remote port (10001) is forwarded to local host (v2xhub) and port: 22222
echo "Open reverse tunnel..."

# open http tunnel, port-forwarding from  REMOTE_PORT to port 22222 (222222: running in v2xhub)
CMD="/usr/bin/ssh -4 -f -i $KEY_FILE -N -o StrictHostKeyChecking=no -o ServerAliveInterval=30 -o ServerAliveCountMax=10 -o UserKnownHostsFile=/dev/null -o LogLevel=quiet -p 22 -R $REMOTE_PORT:localhost:22222 -T $REMOTE_USER@$REMOTE_ADDR" # -f runs ssh in background after it authenticates, -N creates a tunnel without running remote commands to save resources, -T disables pseudo-tty allocation (each article I read on reverse tunneling used these options
ps -x -o pid,cmd | grep "$CMD" > $TEMP_FILE # write the contents of the ps command to a file, it only prints the pid and the command line command used to create the entry
while read -r line; do # read the lines of the file with ps command
    case "$line" in
        grep) # skip the entry that contains grep
            ;;
        *)
            echo $line # everything else should be a process to kill since the lines in the file were filtered by grep
            PID_TO_KILL=`echo $line | cut -d " " -f 1` # the entries are space delimited, the pid is in position 1
            output=$(ps -p "$PID_TO_KILL")
            if [ "$?" -eq 0 ]; then
                kill -9 $PID_TO_KILL
            fi
            ;;
    esac
done < $TEMP_FILE
rm $TEMP_FILE
exec $CMD &> /dev/null  # create the new tunnel



if [ "$?" -eq 0 ]; then   
   if ssh -i $KEY_FILE $REMOTE_USER@$REMOTE_ADDR "sudo lsof -Pi:$REMOTE_PORT -sTCP:LISTEN" >/dev/null; then
        echo "Reverse tunnel is successfully opened!" 
    fi 
fi