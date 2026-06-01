
import numpy as np
import matplotlib.pyplot as plt
import math


class GenerateCSV:
    def __init__(self, file_name):
        self.__fp = open(file_name, 'w')

    def __del__(self):
        self.__fp.close()

    def write_header(self, nq, nv):
        for i in range(nq):
            self.__fp.write('q{},'.format(i))
        for i in range(nv):
            self.__fp.write('v{},'.format(i))
        for i in range(nv):
            self.__fp.write('vdot{},'.format(i))
        self.__fp.write('\n')

    def write_data(self, q, v, vdot):
        for i in list(q):
            self.__fp.write('{},'.format(i))
        for i in list(v):
            self.__fp.write('{},'.format(i))
        for i in list(vdot):
            self.__fp.write('{},'.format(i))
        self.__fp.write('\n')


def plot_scatter(x, y, label):
    plt.title('plot')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(ls='--')
    for i, v in enumerate(y):
        plt.scatter(x, v, label='{}'.format(label[i]))
    plt.legend(loc='upper right')


def plot_line(x, y, label):
    plt.title('plot')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(ls='--')
    for i, v in enumerate(y):
        plt.plot(x, v, ".-", label='{}'.format(label[i]))
    plt.legend(loc='upper right')
