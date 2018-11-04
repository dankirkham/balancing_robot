from multiprocessing import Process
import serial
import csv

class Receiver(Process):
    field_names = [
        'raw_error',
        'error',
        'delta',
        'integral',
        'derivative',
        'steps',
        'steps_to_take_right_now',
        'process_value'
    ]

    def __init__(self, queue, signal, port, baudrate=115200, outfile=None):
        super(Process, self).__init__()
        self.queue = queue
        self.signal = signal
        self.port = port
        self.baudrate = baudrate

        if outfile:
            f = open(outfile, 'w', newline='')

            self.csv = csv.DictWriter(f, fieldnames=Receiver.field_names)
            self.csv.writeheader()

    def run(self):
        ser = serial.Serial(self.port, self.baudrate)

        # ser.open()

        while True:
            line = ser.readline()

            if line:
                fields = line.split()

                if len(fields) == 8:
                    data = {
                        'raw_error': float(fields[0]),
                        "error": float(fields[1]),
                        "delta": float(fields[2]),
                        "integral": float(fields[3]),
                        "derivative": float(fields[4]),
                        "steps": float(fields[5]),
                        "steps_to_take_right_now": float(fields[6]),
                        "process_value": float(fields[7])
                    }

                    if self.csv:
                        self.csv.writerow(data)

                    self.queue.put(data)

            if self.signal.poll():
                break

        ser.close()
