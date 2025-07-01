#!/usr/bin/env python3
import subprocess
import threading
from queue import Queue, Empty

class PlayProcess:
    def __init__(self):

        self.process = None
        self.finish = False
        self.output = []
        self.q = Queue()
        self.thread = None
        self.fraction = 0


    def run(self, bagPath):
        self.process = subprocess.Popen(['rosbag', 'play','--clock', bagPath],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,          # merge stderr with stdout (optional)
                            text=True,                         # decode bytes to string
                            bufsize=1                          # line-buffered
                        )
        self.thread = threading.Thread(target=self.isFinish, args=(self.process, self.q))
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.finish = True
        self.process.terminate()

    def isFinish(self, pipe, queue):
        self.finish = False
        for line in pipe.stdout:
            queue.put(line)
            if(line.strip() == "Done."):
                self.finish = True
                self.out()
                pipe.terminate()

        return self.finish

    def progress_bar(self, current, total, bar_length=30):
        if(self.fraction == 1.0):
            print('✔️')
            self.fraction = self.fraction+1
        elif(self.fraction < 1.0):
            self.fraction = current / total
            filled_length = int(bar_length * self.fraction)
            bar = '▓' * filled_length + '░' * (bar_length - filled_length)
            percent = int(self.fraction * 100)
            print(f'\r[{bar}] {percent}%', end='', flush=True)
        else:
            return

    def out(self):

        now = 0
        total = 100

        if(self.finish):
            self.progress_bar(total, total) #Finish Process!

        try:
            out = self.q.get_nowait()
            out = out.split("Duration: ", 2)
            if(len(out) > 1):
                out = out[1]
                out = out.replace(' ', '')
                out = out.replace('\n', '')
                out = out.split('/', 2)
                try:
                    now = float(out[0]) * 10.0
                    total = float(out[1])
                    if(now > total): now = total
                except ValueError:
                        pass
                self.progress_bar(now, total)
            return self.q.get_nowait()  # Non-blocking
        except Empty:
            return "Without"

