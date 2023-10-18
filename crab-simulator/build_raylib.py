# eval cc yourgame.c $(pkg-config --libs --cflags raylib) -o YourGame

import os
import sys

if len(sys.argv) < 2:
    print("Usage: python build_raylib.py <game_name>")
    sys.exit(1)

game_name = sys.argv[1]
command = f"eval cc {game_name}.c $(pkg-config --libs --cflags raylib) -o {game_name}"
os.system(command)
