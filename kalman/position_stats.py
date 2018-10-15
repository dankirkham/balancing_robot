import argparse
import csv
import statistics

def print_stats(array):
    print('Min\t\t\t{}'.format(min(array)))
    print('Max\t\t\t{}'.format(max(array)))
    print('Mean\t\t\t{}'.format(statistics.mean(array)))
    print('Median\t\t\t{}'.format(statistics.median(array)))
    print('St. Dev.\t\t{}'.format(statistics.stdev(array)))
    print('Variance\t\t{}'.format(statistics.variance(array)))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file')
    args = parser.parse_args()

    f = open(args.file, 'r')
    r = csv.DictReader(f)

    error = []
    raw_error = []

    for row in r:
        error.append(float(row['error']))
        raw_error.append(float(row['raw_error']))

    print('error')
    print_stats(error)
    print('------------------')
    print('raw_error')
    print_stats(raw_error)

if __name__ == '__main__':
    main()
