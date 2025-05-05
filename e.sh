#!/bin/bash
# r.sh

gcc -o test main.c dbc.c OCV_SOC_T.c -lpthread -lm

if [ $? -eq 0 ]; then
    echo -e "🍀🍀Build complete🍀🍀\n"
else
    echo -e "❌❌Build failed!❌❌\n"
    exit 1
fi
