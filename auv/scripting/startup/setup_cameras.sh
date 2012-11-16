#!/bin/bash
sudo rmmod gspca_ov534
sudo modprobe ov534

for file in /dev/video0 /dev/video1 /dev/video2
do
    v4lctl setattr -c $file VFlip on
    v4lctl setattr -c $file HFlip on
    v4lctl setattr -c $file 'Auto Gain' off
    v4lctl setattr -c $file 'Main Gain' 15
done
