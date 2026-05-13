#!/usr/bin/env bash

if [ -z "${WHISPER}" ]; then
    WHISPER="whisper"
fi
$WHISPER --server server.txt test.elf &
sleep 0.1 # wait for server to init
port=$(awk '{print $2}' server.txt)
./client $port
