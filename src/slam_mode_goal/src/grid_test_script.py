#! /bin/usr/env python

if __name__ == "__main__":
    with open("local_costmap_data.txt", "r") as read_file:
        read_data = read_file.read().split(',')
    print(len(read_data))
