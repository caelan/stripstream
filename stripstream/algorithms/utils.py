from stripstream.algorithms.search.fast_downward import get_fast_downward, has_fd
from stripstream.algorithms.search.bfs import get_bfs
from stripstream.pddl.logic.predicates import Predicate
from stripstream.pddl.objects import Constant, OBJECT

BFS, FAST_DOWNWARD, FAST_FORWARD, LAPKT, PYPLANNERS = 'bfs', 'fd', 'ff', 'lapkt', 'pyplanners'
INCREMENTAL, FOCUSED = 'incremental', 'focused'

if has_fd():
    DEFAULT_SEARCH = get_fast_downward()
else:
    DEFAULT_SEARCH = get_bfs()


class AbstractConstant(Constant):
    _num = 0
    _template = '_ab_%s_%s'


def make_abstract_constant(ty, shared=False):
    name = AbstractConstant._template % (ty.name, AbstractConstant._num)
    if shared:
        name = '_sh_%s_%s' % (ty.name, AbstractConstant._num)
    AbstractConstant._num += 1
    return AbstractConstant(name, ty)

Concrete = Predicate('_concrete', [OBJECT])
