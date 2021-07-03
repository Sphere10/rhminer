#!/bin/bash
while true
do
    ./rhminer -s stratum+tcp://mine.pool.pascalpool.org:3334 -su 1300378-87.0.Donations -cpu -cputhreads 4 -r 40
    sleep 5s
done




