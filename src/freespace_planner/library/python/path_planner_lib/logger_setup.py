from datetime import datetime
import time
from termcolor import colored


class Logger:
    # filename = "../log/log.txt"  # Default value will be overwritten

    colors = {"SUCCESS": "green",
              "DEBUG": "magenta",
              "INFO": "white",
              "TIME": "white",
              "ERROR": "red",
              "WARN": "yellow"}

    module = "UNKNOWN"

    timestamp_str = None
    tabs_expanded = 10

    def __init__(self, obj):
        data_time_obj = datetime.now()
        timestamp_str = data_time_obj.strftime("%d-%m-%Y-%H_%M_%S_%f")
        # set class object
        if Logger.timestamp_str is None:
            Logger.timestamp_str = timestamp_str

        # set log filename
        Logger.filename = "../log/log_" + Logger.timestamp_str + ".txt"
        self.module = obj.__module__

    @staticmethod
    def create_text(type, m, output, var=None):
        current_time = time.time()
        if var is not None:
            return ("[" + type + "] ["+str(current_time)+"] [" + m + "]:\t" + output + " = " + str(var)).expandtabs(Logger.tabs_expanded)
        else:
            return ("[" + type + "] ["+str(current_time)+"] [" + m + "]:\t" + output).expandtabs(Logger.tabs_expanded)

    def log_info(self, output, var=None):
        log_type = "INFO"
        text = Logger.create_text(log_type, self.module, output, var)
        print(text, flush=True)
        # Logger.print2file(text)

    @staticmethod
    def log_timing(output, module, var=None):
        log_type = "TIME"
        c = Logger.colors[log_type]
        text = Logger.create_text(log_type, module, output, var)
        print(colored(text, c), flush=True)
        # Logger.print2file(text)

    def log_success(self, output, var=None):
        log_type = "SUCCESS"
        c = Logger.colors[log_type]
        text = Logger.create_text(log_type, self.module, output, var)
        print(colored(text, c), flush=True)
        # Logger.print2file(text)

    def log_error(self, output, var=None):
        log_type = "ERROR"
        c = Logger.colors[log_type]
        text = Logger.create_text(log_type, self.module, output, var)
        print(colored(text, c))
        # Logger.print2file(text)

    def log_debug(self, output, var=None):
        log_type = "DEBUG"
        c = Logger.colors[log_type]
        text = Logger.create_text(log_type, self.module, output, var)
        print(colored(text, c), flush=True)
        # Logger.print2file(text)

    def log_warning(self, output, var=None):
        log_type = "WARN"
        c = Logger.colors[log_type]
        text = Logger.create_text(log_type, self.module, output, var)
        print(colored(text, c), flush=True)
        # Logger.print2file(text)
