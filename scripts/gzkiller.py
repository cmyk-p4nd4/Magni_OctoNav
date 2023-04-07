#!/usr/bin/env python3

import sys, signal, subprocess
import time

def main() -> None:
    handler = signal.signal(signal.SIGINT, sig_handler)
    sys.argv = ' '.join(sys.argv[1:])
    subprocess.call(sys.argv, shell=True)

def sig_handler(sig: signal.Signals, frame) -> None:
    time.sleep(1)
    subprocess.run("killall -9 gzclient && killall -9 gzserver", shell=True)

if __name__ == "__main__":
    main()