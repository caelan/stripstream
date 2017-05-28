from stripstream.algorithms.search.fast_downward import get_fast_downward, has_fd
from stripstream.algorithms.search.fast_forward import get_fast_forward, has_ff
from stripstream.algorithms.search.lapkt import get_lakpt, has_lapkt

BFS, FAST_DOWNWARD, FAST_FORWARD, LAPKT, PYPLANNERS = 'bfs', 'fd', 'ff', 'lapkt', 'pyplanners'
INCREMENTAL, FOCUSED = 'incremental', 'focused'

DEFAULT_SEARCH_NAME = BFS
if has_fd():
    DEFAULT_SEARCH_NAME = FAST_DOWNWARD
elif has_ff():
    DEFAULT_SEARCH_NAME = FAST_FORWARD
elif has_lapkt():
    DEFAULT_SEARCH_NAME = LAPKT
print 'Using', DEFAULT_SEARCH_NAME, 'as the default search implementation'

if DEFAULT_SEARCH_NAME == FAST_DOWNWARD:
    DEFAULT_SEARCH = get_fast_downward()
elif DEFAULT_SEARCH_NAME == FAST_FORWARD:
    DEFAULT_SEARCH = get_fast_forward()
elif DEFAULT_SEARCH_NAME == LAPKT:
    DEFAULT_SEARCH = get_lakpt()
elif DEFAULT_SEARCH_NAME == BFS:
    from stripstream.algorithms.search.bfs import get_bfs
    DEFAULT_SEARCH = get_bfs()
else:
    raise ValueError('Unexpected search type %s' % DEFAULT_SEARCH_NAME)
