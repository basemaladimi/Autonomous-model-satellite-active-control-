cmake_minimum_required(VERSION 3.22.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(print_float)

target_sources(app PRIVATE src/main.c)