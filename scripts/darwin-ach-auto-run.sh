#!/bin/bash

DO_EXIT=0

while [ 1 -gt $DO_EXIT ]
do
  SERVER_STATUS=$(darwin-ach status -n server)
  if [ $SERVER_STATUS -eq 0 ]
  then
    darwin-ach stop server
    darwin-ach kill server
    darwin-ach ach reset
    darwin-ach start server
  else
    sleep 5
  fi
done
