#!/usr/bin/env python3

from env import Environment

if __name__ == "__main__":
    env = Environment()
    while True:
        env.step()