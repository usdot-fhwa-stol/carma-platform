#! /bin/bash

# Searches log files for the plugins being instantiated to see if there are any duplicates.

if [ $# = 0 ]
then
    echo "Usage is: plugin-search.bash <log file(s)>"
    exit 1
fi

#loop on log files
for file in $@;
do
    :

    #echo "Scanning $file"

    #count the number of restarts of a route in this log (we expect each plugin to show
    # up for each restart plus once initially)
    let num_restarts=($(grep "Arbitrator processing event: CLEAN_RESTART" $file | wc -l))
    #echo "Num restarts = $num_restarts"

    #get all instatiated plugins for this run
    plugins=($(awk 'BEGIN { FS=":"; } /will initialize plugin/ { print $5; } END { FS=" "; }' $file))

    #sort this list
    plugins=($(for each in ${plugins[@]}; do echo $each; done | sort))
    #echo "Sorted list:"
    #for k in ${plugins[@]}; do echo $k; done

    #look for duplicates
    let i=0
    prev_base="B"
    for base in "${plugins[@]}"
    do
        :
        #echo "new base = $base"

        if [ $base != $prev_base ]
        then

            found="false"
            let num_matches=0
            let top=${#plugins[@]}-1
            if [ $i -lt $top ]
            then
                let j=0
                for tgt in "${plugins[@]}"
                do
                    :

                    if [ $j -gt $i ] && [ $found = "false" ]
                    then
                        #echo "Testing i = $i, j = $j, base = $base, tgt = $tgt"
                        if [ $base = $tgt ]
                        then
                            let num_matches=$num_matches+1
                            if [ $num_matches -gt $num_restarts ]
                            then
                                echo "----- Found an excessive match in $file: $base"
                                found="true"
                            fi
                        fi
                    fi
                    let j=$j+1
                done
            fi

        fi
        let i=$i+1
        prev_base=$base
    done
done
