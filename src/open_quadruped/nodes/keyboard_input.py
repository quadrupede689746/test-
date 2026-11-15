"""Nonblocking terminal key reader."""
import os, select, sys

def read_key(timeout=.02):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return os.read(sys.stdin.fileno(), 1).decode(errors='ignore') if ready else None
