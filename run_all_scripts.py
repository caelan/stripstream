import scripts
import os
import pkgutil
import importlib
from inspect import getmembers, isfunction, getargspec
from stripstream.utils import SEPARATOR

IMPORT_ONLY = True

pkgpath = os.path.dirname(scripts.__file__)
for _, module_name, is_dir in pkgutil.iter_modules([pkgpath]):
    if not is_dir:
        print SEPARATOR
        location = '.'.join([scripts.__name__, module_name])
        print location
        module = importlib.import_module(location)
        #module = __import__(location)

        if not IMPORT_ONLY:
            for name, value in getmembers(module):
                if isfunction(value) and name == 'main' and getargspec(value)[0] == []:
                    value()
