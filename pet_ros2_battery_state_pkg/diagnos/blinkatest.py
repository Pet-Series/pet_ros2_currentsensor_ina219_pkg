#!/usr/bin/env python3'
# coding = utf-8
####################################################################################
# System test for Adafruit BLINKA.
# Make sure that all Python3 package are installed and working.
# Adjust access rights for the I2C and/or GPIO-pins.
#
# ---- install
# $ sudo apt-get install python3-pip
# $ sudo apt-get install i2c-tools 
# $ sudo apt-get install libgpiod-dev
# $ sudo apt-get install RPi.GPIO
# $ sudo pip3 install board 
# $ sudo pip3 install smbus2
# $ sudo apt-get install adafruit-blinka
# ($ sudo pip3 install --force-reinstall adafruit-blinka )
#
# ---- Access to i2c and GPIO
# $ sudo chmod a+rw /dev/i2c-1
# $ sudo groupadd i2c
# $ sudo usermod -aG i2c pi
# $ sudo usermod -a -G gpio pi
#
# ---- Run the script
# $ sudo python3 blinkatest.py
# $ python3 blinkatest.py      # Might work...
#
import board
import digitalio
import busio

print("Hello blinka!")

# Try to create a Digital GPIO-input object.
pin = digitalio.DigitalInOut(board.D4)
print("Digital IO ok!")

# Try to create an I2C object.
i2c = busio.I2C(board.SCL, board.SDA)
print("I2C ok!")

# Try to create an SPI object.
spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
print("SPI ok!")

print("done!")