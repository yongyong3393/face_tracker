#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pigpio
import time

ERROR = 0xFE
PIN = 17

pi = pigpio.pi()
if not pi.connected:
    exit()

pi.set_mode(PIN, pigpio.INPUT)
pi.set_pull_up_down(PIN, pigpio.PUD_UP)

def get_key():
    byte = [0, 0, 0, 0]
    if not ir_start():
        time.sleep(0.11)  # One message frame lasts 108 ms.
        return ERROR
    else:
        for i in range(4):
            byte[i] = get_byte()
        if byte[0] + byte[1] == 0xFF and byte[2] + byte[3] == 0xFF:
            return byte[2]
        else:
            return ERROR

def ir_start():
    time_falling_edge = [0, 0]
    time_rising_edge = 0
    time_span = [0, 0]

    try:
        print("Waiting for first FALLING edge...")
        pi.wait_for_edge(PIN, pigpio.FALLING_EDGE)
        time_falling_edge[0] = time.time()
        print(f"First FALLING edge detected at {time_falling_edge[0]}")

        print("Waiting for RISING edge...")
        pi.wait_for_edge(PIN, pigpio.RISING_EDGE)
        time_rising_edge = time.time()
        print(f"RISING edge detected at {time_rising_edge}")

        print("Waiting for second FALLING edge...")
        pi.wait_for_edge(PIN, pigpio.FALLING_EDGE)
        time_falling_edge[1] = time.time()
        print(f"Second FALLING edge detected at {time_falling_edge[1]}")
    except RuntimeError as e:
        print(f"Error waiting for edge: {e}")
        return False

    time_span[0] = time_rising_edge - time_falling_edge[0]
    time_span[1] = time_falling_edge[1] - time_rising_edge
    print(f"time_span[0]: {time_span[0]}, time_span[1]: {time_span[1]}")

    if 0.0085 < time_span[0] < 0.0095 and 0.004 < time_span[1] < 0.005:
        print("Start signal confirmed")
        return True
    else:
        print("Start signal not detected")
        return False

def get_byte():
    byte = 0
    time_rising_edge = 0
    time_falling_edge = 0
    time_span = 0

    for i in range(8):
        try:
            pi.wait_for_edge(PIN, pigpio.RISING_EDGE)
            time_rising_edge = time.time()

            pi.wait_for_edge(PIN, pigpio.FALLING_EDGE)
            time_falling_edge = time.time()

            time_span = time_falling_edge - time_rising_edge
            print(f"Bit {i}: time_span = {time_span}")

            if 0.0015 < time_span < 0.0019:
                byte |= 1 << i
        except RuntimeError as e:
            print(f"Error waiting for edge: {e}")
            return ERROR

    return byte

print('IR Remote Test Start...')
try:
    while True:
        key = get_key()
        if key != ERROR:
            print(f"Get the key: 0x{key:02x}")
except KeyboardInterrupt:
    pi.stop()
