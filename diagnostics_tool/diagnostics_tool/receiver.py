from multiprocessing import Process
import serial

class Receiver(Process):
    def __init__(self, queue, signal, port, baudrate=115200):
        super(Process, self).__init__()
        self.queue = queue
        self.signal = signal
        self.port = port
        self.baudrate = baudrate

    def run(self):
        ser = serial.Serial(self.port, self.baudrate)

        # ser.open()

        while True:
            line = ser.readline()

            if line:
                fields = line.split()

                if len(fields) == 7:
                    data = {
                        "error": float(fields[0]),
                        "delta": float(fields[1]),
                        "integral": float(fields[2]),
                        "derivative": float(fields[3]),
                        "steps": float(fields[4]),
                        "steps_to_take_right_now": float(fields[5]),
                        "process_value": float(fields[6])
                    }

                    self.queue.put(data)

            if self.signal.poll():
                break

        ser.close()
