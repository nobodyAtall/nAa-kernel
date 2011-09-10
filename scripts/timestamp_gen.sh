#!/bin/bash

# Generate the build timestamp based on the last commit in kernel git if available or build time otherwise.
timest=`git log -1 --pretty=format:%ad 2> /dev/null | sed 's/ *$//; s/ [^ ]*$//'`
if [ $? -ne 0 -o -z "$timest" ]; then
    date
else
    echo $timest
fi

