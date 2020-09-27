#!/usr/bin/env python3

import sys
import csv

import numpy as np
import matplotlib.pyplot as plt


def main(file1, file2):
    with open(file1) as f:
        data1 = [[float(j) for j in i] for i in list(csv.reader(f))]

    if file2 is not None:
        with open(file2) as f:
            data2 = [[float(j) for j in i] for i in list(csv.reader(f))]

    cols = list(zip(*data1))
    x = cols[0]
    ys = cols[1:]

    fig, axes = plt.subplots(len(ys), 2 if file2 is not None else 1, sharey="row")
    axes = np.atleast_2d(axes)

    if file2 is None:
        axes = axes.transpose()

    for i, y in enumerate(ys):
        axes[i, 0].plot(x, y)

    if file2 is not None:
        cols = list(zip(*data2))
        x = cols[0]
        ys = cols[1:]

        for i, y in enumerate(ys):
            axes[i, 1].plot(x, y)

    plt.show()


if __name__ == "__main__":
    if len(sys.argv) <= 1:
        print("Usage: ./plot_data.py <file1.csv> [<file2.csv>]")
    else:
        file1 = sys.argv[1]
        file2 = sys.argv[2] if len(sys.argv) > 2 else None
        main(file1, file2)
