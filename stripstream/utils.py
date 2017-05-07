import os
import random
import shutil
import numpy
import pickle
from random import shuffle

INF = float('inf')
SEPARATOR = '\n' + 85 * '-' + '\n'


def separator(n=85):
    return '\n' + n * '-' + '\n'


def header(s, n=10):
    return '\n' + n * '-' + s + n * '-' + '\n'


def set_deterministic(seed=0):
    random.seed(seed)
    numpy.random.seed(seed)


def implies(a, b):
    return not a or b


def flatten(iterable_of_iterables):
    return (item for iterables in iterable_of_iterables for item in iterables)


def irange(start, end, step=1):
    n = start
    while n < end:
        yield n
        n += step


def argmin(function, sequence):
    values = list(sequence)
    scores = [function(x) for x in values]
    return values[scores.index(min(scores))]


def first(function, iterable):
    for item in iterable:
        if function(item):
            return item
    return None


def random_sequence(sequence):
    indices = range(len(sequence))
    shuffle(indices)
    for i in indices:
        yield sequence[i]


def read(filename):
    with open(filename, 'r') as f:
        return f.read()


def write(filename, string):
    with open(filename, 'w') as f:
        f.write(string)


def write_pickle(filename, data):
    with open(filename, 'wb') as f:
        pickle.dump(data, f)


def read_pickle(filename):
    with open(filename, 'rb') as f:
        return pickle.load(f)


def ensure_dir(f):
    d = os.path.dirname(f)
    if not os.path.exists(d):
        os.makedirs(d)


def safe_remove(p):
    if os.path.exists(p):
        os.remove(p)


def remove_dir(d):
    if os.path.exists(d):
        shutil.rmtree(d)


GRAPH_VIZ_COLORS = ['AntiqueWhite', 'Aquamarine', 'Beige', 'Bisque', 'Black', 'BlanchedAlmond', 'Blue', 'BlueViolet',
                    'Brown', 'Burlywood', 'CadetBlue', 'Chartreuse', 'Chocolate', 'Coral', 'CornflowerBlue', 'Cornsilk',
                    'Crimson', 'Cyan', 'DarkBlue', 'DarkCyan', 'DarkGoldenrod', 'DarkGray', 'DarkGreen', 'DarkKhaki',
                    'DarkMagenta', 'DarkOliveGreen', 'DarkOrange', 'DarkOrchid', 'DarkRed', 'DarkSalmon', 'DarkSeaGreen',
                    'DarkSlateBlue', 'DarkSlateGray', 'DarkTurquoise', 'DarkViolet', 'DeepPink', 'DeepSkyBlue', 'DimGray',
                    'DodgerBlue', 'Firebrick', 'ForestGreen', 'Fuchsia', 'Gainsboro', 'Gold', 'Goldenrod', 'Gray', 'Green',
                    'GreenYellow', 'HotPink', 'IndianRed', 'Indigo', 'Khaki', 'Lavender', 'LavenderBlush', 'LawnGreen',
                    'LemonChiffon', 'LightBlue', 'LightCoral', 'LightCyan', 'LightGoldenrodYellow', 'LightGray',
                    'LightGreen', 'LightPink', 'LightSalmon', 'LightSeaGreen', 'LightSkyBlue', 'LightSlateGray',
                    'LightSteelBlue', 'Lime', 'LimeGreen', 'Magenta', 'Maroon', 'MediumAquamarine', 'MediumBlue',
                    'MediumOrchid', 'MediumPurple', 'MediumSeaGreen', 'MediumSlateBlue', 'MediumSpringGreen',
                    'MediumTurquoise', 'MediumVioletRed', 'MidnightBlue', 'MistyRose', 'Moccasin', 'NavajoWhite',
                    'Navy', 'OldLace', 'OliveDrab', 'Orange', 'OrangeRed', 'Orchid', 'PaleGoldenrod', 'PaleGreen',
                    'PaleTurquoise', 'PaleVioletRed', 'PapayaWhip', 'PeachPuff', 'Peru', 'Pink', 'Plum', 'PowderBlue',
                    'Purple', 'Red', 'RosyBrown', 'RoyalBlue', 'RoyalBlue', 'Salmon', 'SandyBrown', 'SeaGreen', 'Sienna',
                    'Silver', 'SkyBlue', 'SlateBlue', 'SlateGray', 'SpringGreen', 'SteelBlue', 'Tan', 'Teal', 'Thistle',
                    'Tomato', 'Turquoise', 'Violet', 'Wheat', 'Yellow', 'YellowGreen']

GRAPH_VIZ_SHAPES = ['box', 'oval', 'square', 'circle']
