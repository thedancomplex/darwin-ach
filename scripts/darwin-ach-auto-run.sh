#!/bin/bash

DO_EXIT=0

while [ 1 -gt $DO_EXIT ]
do
  SERVER_STATUS=$(darwin-ach status -n server)
  if [ $SERVER_STATUS -eq 0 ]
  then
    darwin-ach stop server no_wait
    darwin-ach kill server no_wait
    darwin-ach ach reset no_wait
    darwin-ach start server no_wait
  else
    sleep 5
  fi
done
