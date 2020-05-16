#! /usr/bin/env python

from pynput import keyboard

COMB_START = [
    {keyboard.Key.ctrl, keyboard.KeyCode(char='g')},
    {keyboard.Key.ctrl, keyboard.KeyCode(char='G')}
]

print(type(COMB_START[0]))
