#!/bin/bash

# Read topics from the file
while IFS= read -r topic; do
    topics="$topics $topic"
done < topics.txt

# Record topics to a bag file
ros2 bag record $topics -o bag_1
