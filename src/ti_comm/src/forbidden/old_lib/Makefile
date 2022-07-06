# Compiler Option
CC = g++

# Compiler Flags
CFLAGS = -g -Wall

default: all

all: amp_serial_jetson.o libserialport-0.1.0/serialport.o libserialport-0.1.0/linux_termios.o
	$(CC) $(CFLAGS) -o amp_serial_jetson amp_serial_jetson.o libserialport-0.1.0/serialport.o libserialport-0.1.0/linux_termios.o -ludev

amp_serial_jetson.o: amp_serial_jetson.cpp amp_serial_jetson.h util.h amp_err.h
	$(CC) $(CFLAGS) -c amp_serial_jetson.cpp

sample_publisher: sample_publisher.o
	$(CC) $(CFLAGS) -o sample_publisher sample_publisher.o

sample_publisher.o: sample_publisher.cpp
	$(CC) $(CFLAGS) -c sample_publisher.cpp
