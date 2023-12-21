import time
import matplotlib.pyplot as plt

class Timer():
    def __init__(self, start = 0.0):
        self.start = start
        self.data = []
        self.time = []
        self.total_time = 0

    def start_timer(self, now = time.time()):
        self.start = now
    
    def stop_timer(self, now = time.time()):
        delta_time = now - self.start
        self.data.append(delta_time)
        self.time.append(now)
        self.total_time += delta_time
    
class Profiler():
    
    def __init__(self, initial_time = 0.0):
        self.timers = {}
        self.initial_time = initial_time

    def add_timers(self, timers_name : list):
        for timer_name in timers_name:
            self.add_timer(timer_name)

    def add_timer(self, timer_name : str):
        self.timers[timer_name] = Timer()

    def start_timer(self, timer_name : str, now = time.time()):
        self.timers[timer_name].start_timer(now - self.initial_time)

    def stop_timer(self, timer_name : str, now = time.time()):
        self.timers[timer_name].stop_timer(now - self.initial_time)

    def print_timers_total_time(self):
        print("[Profiler] Total Time")
        for timer_name, timer in self.timers.items():
            print(timer_name + " : " + str(timer.total_time))

    def plot_timers(self, show_plot : bool=True):
        fig = plt.figure()
        
        for timer_name, timer in self.timers.items():
            plt.plot(timer.time,timer.data, label=timer_name)

        plt.legend(loc="upper left")
        plt.ylabel('Duration [s]')
        plt.xlabel('Time [s]')
        plt.title('Profiler')
        
        if show_plot:
            plt.show()

        return fig

