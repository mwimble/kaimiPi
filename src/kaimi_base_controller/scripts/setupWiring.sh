#!/bin/sh
echo "Setting up pin 12 as GPIO input"
gpio export 12 input
gpio export 12 up
gpio export 17 input
gpio export 18 output
gpio export 18 up
