from .receiver import Receiver
from multiprocessing import Queue, Pipe
import argparse
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port')
    parser.add_argument('--baudrate', type=int, default=115200)

    args = parser.parse_args()

    queue = Queue()
    signal1, signal2 = Pipe()

    receiver = Receiver(queue, signal1, args.port, baudrate=args.baudrate)

    receiver.start()

    fig = plt.figure()
    plt.axis([0, 1000, -90, 90])
    fig.suptitle('Angle Error')
    plt.xlabel('elapsed time (s)')
    plt.ylabel('angle error (°)')

    sample_count = 0
    errors = []

    error_line, = plt.plot(errors, 'k')

    while True:
        message = queue.get()
        print(message)

        errors.append(message['error'])
        sample_count += 1

        error_line.set_xdata(range(sample_count))
        error_line.set_ydata(errors)

        # plt.plot(errors, 'k')

        plt.pause(0.05)

    plt.show()

if __name__ == "__main__":
    main()
