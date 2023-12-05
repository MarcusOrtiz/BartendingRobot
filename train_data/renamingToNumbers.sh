#!/bin/bash

cd ./images/train

a=1
for i in *.png; do
  new=$(printf "%03d.png" "$a") # Adjust the format as needed
  mv -- "$i" "$new"
  let a=a+1
done