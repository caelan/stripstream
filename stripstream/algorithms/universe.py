from collections import defaultdict, namedtuple
from time import time
from operator import attrgetter

from stripstream.algorithms.utils import AbstractConstant, Concrete
from stripstream.utils import SEPARATOR
from stripstream.pddl.logic.atoms import Initialize, Atom
from stripstream.pddl.logic.predicates import Predicate, TotalCost, Function
from stripstream.pddl.objects import Parameter, Constant
from stripstream.pddl.operators import Action, Axiom
from stripstream.utils import INF


from stripstream.algorithms.instantiation import parameter_product

ValueStats = namedtuple(
    'ValueStats', ['stream', 'call_time', 'prior_time', 'prior_calls'])

DEFAULT_COST = INF


def get_key(atom):
    return atom.predicate, tuple(arg.type for arg in atom.args)


class Universe(object):
    domain_name = 'stripstream'
    problem_name = domain_name

    def __init__(self, problem, use_ground, make_stream_instances=False, make_action_instances=False, hierarchical=False, verbose=False):
        self.start_time = time()
        self.problem = problem
        self.use_ground = use_ground
        self.make_stream_instances = make_stream_instances or not use_ground
        self.make_action_instances = make_action_instances
        self.hierarchical = hierarchical
        self.verbose = verbose

        self.predicates = {}
        self.functions = []
        self.name_to_object = {}
        self.type_to_objects = {}
        self.object_to_type = {}
        self.name_to_action = {}
        self.axioms = []
        self.initial_cost = 0

        self.initial_atoms = set()
        self.stream_atoms = set()
        self.temporary_atoms = set()
        self.perm_functions = {}
        self.temp_functions = {}

        self.goal_formula = problem.goal_literals

        self.fluent_predicates = set()

        self.derived_predicates = {}

        self.stream_predicates = set()

        self.predicate_to_atoms = defaultdict(set)
        self.value_producers = defaultdict(list)

        self.calls = 0
        self.empty = 0
        self.intersect = 0
        self.search_time = 0
        self.resets = 0
        self.enumerated = 0
        self.test_calls = 0
        self.iterations = 0

        self.new_streams = []
        self.new_problem = False
        self.temp_blocked = set()
        self.perm_blocked = set()
        self.new_instances = []
        self.action_instances = set()

        self.cond_streams = []
        self.cond_conditions = defaultdict(list)
        self.cond_params = defaultdict(list)

        self.act_conditions = defaultdict(list)
        self.act_params = defaultdict(list)
        self.act_stream_conditions = {}
        self.act_free_params = {}
        if self.make_action_instances:
            assert not self.hierarchical

            print 'Warning: make_action_instances only currently supports conjunctive static predicates'

            stream_predicates = problem.get_stream_predicates()
            for act in problem.operators:

                self.act_stream_conditions[act] = filter(
                    lambda c: c.predicate in stream_predicates, act.condition.get_atoms())
                for con in self.act_stream_conditions[act]:
                    self.act_conditions[con.predicate].append((act, con))
                con_params = {a for con in self.act_stream_conditions[
                    act] for a in con.args}
                self.act_free_params[act] = [
                    p for p in act.parameters if p not in con_params]
                for param in self.act_free_params[act]:
                    self.act_params[param.type].append((act, param))

        for cs in problem.cond_streams:
            self.add_cond_stream(cs)
        for atom in problem.initial_atoms:
            if isinstance(atom, Initialize):
                self.add_perm_initialize(atom)
            else:
                self.add_initial_atom(atom)
        self.add_formula(self.goal_formula)
        for obj in problem.objects:
            self.add_object(obj)
        for operator in problem.operators:
            if isinstance(operator, Action):
                self.add_action(operator)
            elif isinstance(operator, Axiom):
                self.add_axiom(operator)

        if self.verbose:
            print 'Fluents:', self.fluent_predicates
            print 'Derived:', set(self.derived_predicates)
            print 'Stream:', self.stream_predicates
            print 'Static:', self.static_predicates

        self.stream_atoms.update(self.initial_stream_atoms())

    @property
    def fixed_predicates(self):
        return set(self.predicates.values()) - self.fluent_predicates - set(self.derived_predicates)

    @property
    def static_predicates(self):
        return self.fixed_predicates - self.stream_predicates

    @property
    def actions(self):
        return self.name_to_action.values()

    def get_object(self, name):
        return self.name_to_object[name]

    def get_action(self, name):
        return self.name_to_action[name]

    def is_internal(self, atom):
        return atom.predicate in [Concrete]

    def is_static(self, atom):
        return atom.predicate in self.static_predicates

    def is_fluent(self, atom):
        return atom.predicate in self.fluent_predicates

    def is_derived(self, atom):
        return atom.predicate in self.derived_predicates

    def is_stream(self, atom):
        return atom.predicate in self.stream_predicates

    def add_stream(self, cond_stream, combo):

        stream = cond_stream(combo)
        self.new_streams.append(stream)

    def add_cond_stream(self, cs):
        for atom in cs.conditions + cs.effects:

            if isinstance(atom.predicate, Predicate):
                self.add_predicate(atom.predicate)
                if self.is_derived(atom):
                    raise ValueError(
                        '%s is a stream and derived predicate' % atom.predicate)
                if self.is_fluent(atom):
                    raise ValueError(
                        '%s is a stream and fluent predicate' % atom.predicate)
                self.stream_predicates.add(atom.predicate)
        if self.make_stream_instances:
            for con in cs.conditions:
                self.cond_conditions[con.predicate].append((cs, con))
            for param in cs.free_params:
                self.cond_params[param.type].append((cs, param))
            if len(cs.inputs) == 0:
                self.add_stream(cs, tuple())

    def add_type(self, ty):
        if ty not in self.type_to_objects:
            self.type_to_objects[ty] = set()

    def add_object(self, obj):
        assert isinstance(obj, Constant)

        self.add_type(obj.type)
        if obj in self.type_to_objects[obj.type]:
            return False

        self.new_problem = True
        self.name_to_object[obj.name] = obj
        self.type_to_objects[obj.type].add(obj)
        if obj in self.object_to_type:
            raise RuntimeError('Cannot share objects across types: %s vs %s' % (
                obj.type, self.object_to_type[obj]))
        assert obj not in self.object_to_type
        self.object_to_type[obj] = obj.type
        if self.use_ground and not isinstance(obj, AbstractConstant):
            self.add_initial_atom(Concrete(obj))

        if self.make_stream_instances and obj.type in self.cond_params:
            for cs, param in self.cond_params[obj.type]:
                con_values = [self.predicate_to_atoms[cond.predicate]
                              for cond in cs.conditions]
                param_values = [self.type_to_objects.get(other.type, []) if param != other else [
                    obj] for other in cs.free_params]
                for param_map in parameter_product(cs.conditions, con_values, cs.free_params, param_values):
                    combo = tuple(param_map[p] for p in cs.inputs)
                    if combo not in cs.streams:
                        self.add_stream(cs, combo)

        if self.make_action_instances and obj.type in self.act_params:
            for act, param in self.act_params[obj.type]:
                con_values = [self.predicate_to_atoms[cond.predicate]
                              for cond in self.act_stream_conditions[act]]
                param_values = [self.type_to_objects.get(other.type, []) if param != other else [
                    obj] for other in self.act_free_params[act]]
                for param_map in parameter_product(self.act_stream_conditions[act], con_values, self.act_free_params[act], param_values):
                    instance = act.instantiate(
                        tuple(param_map[p] for p in act.parameters))
                    if instance not in self.action_instances:
                        self.action_instances.add(instance)
                        self.new_instances.append(instance)

        return True

    def add_producer(self, value, stream):
        self.value_producers[value].append(ValueStats(
            stream, stream.call_times[-1], stream.calls, stream.call_time))

    def get_min_attr(self, value, attr):
        streams = map(attrgetter('stream'), self.value_producers[value])
        attrs = map(attrgetter(attr), self.value_producers[value])
        if len(streams) == 0:
            return None, 0
        index = attrs.index(min(attrs))
        return streams[index], attrs[index]

    def add_predicate(self, predicate):
        assert isinstance(predicate, Predicate)
        if predicate.name in self.predicates:
            return
        self.predicates[predicate.name] = predicate

    def add_atom(self, atom):
        assert isinstance(atom, Atom) and not atom.is_function()
        self.add_predicate(atom.predicate)
        for arg in atom.args:

            if not isinstance(arg, Parameter):
                self.add_object(arg)

    def add_initial_atom(self, atom):
        if atom in self.initial_atoms:
            return False
        self.new_problem = True
        self.add_atom(atom)
        self.initial_atoms.add(atom)
        if self.is_stream(atom):
            self.stream_atoms.add(atom)
        self.predicate_to_atoms[atom.predicate].add(atom)

        if self.make_stream_instances and atom.predicate in self.cond_conditions:
            for cs, con in self.cond_conditions[atom.predicate]:
                con_values = [self.predicate_to_atoms[other.predicate]
                              if other != con else [atom] for other in cs.conditions]
                param_values = [self.type_to_objects.get(
                    other.type, []) for other in cs.free_params]
                for param_map in parameter_product(cs.conditions, con_values, cs.free_params, param_values):
                    combo = tuple(param_map[p] for p in cs.inputs)
                    if combo not in cs.streams:
                        self.add_stream(cs, combo)

        if self.make_action_instances and atom.predicate in self.act_conditions:
            for act, con in self.act_conditions[atom.predicate]:
                con_values = [self.predicate_to_atoms[other.predicate] if other != con else [
                    atom] for other in self.act_stream_conditions[act]]
                param_values = [self.type_to_objects.get(
                    other.type, []) for other in self.act_free_params[act]]
                for param_map in parameter_product(self.act_stream_conditions[act], con_values, self.act_free_params[act], param_values):
                    instance = act.instantiate(
                        tuple(param_map[p] for p in act.parameters))
                    if instance not in self.action_instances:
                        self.action_instances.add(instance)
                        self.new_instances.append(instance)
        return True

    def reset_temporary(self, stream_actions):
        assert self.use_ground
        self.temporary_atoms = set()
        self.predicate_to_temp_atoms = defaultdict(set)
        self.sa_conditions = defaultdict(list)
        self.stream_action_instances = set()
        self.new_stream_actions = []
        for act in stream_actions:
            for con in act.conditions:
                self.sa_conditions[get_key(con)].append((act, con))
            if len(act.parameters) == 0:
                instance = act.instantiate(tuple())
                self.stream_action_instances.add(instance)
                self.new_stream_actions.append(instance)

    def add_temporary_atom(self, atom):
        key = get_key(atom)
        self.add_atom(atom)
        self.temporary_atoms.add(atom)
        self.predicate_to_temp_atoms[key].add(atom)
        for act, con in self.sa_conditions.get(key, []):
            con_values = [self.predicate_to_temp_atoms[get_key(other)] if other != con else [
                atom] for other in act.conditions]
            for param_map in parameter_product(act.conditions, con_values, [], {}):
                instance = act.instantiate(
                    tuple(param_map[p] for p in act.parameters))
                if instance not in self.stream_action_instances and instance not in self.temp_blocked and instance not in self.perm_blocked:
                    self.stream_action_instances.add(instance)
                    self.new_stream_actions.append(instance)

    def add_function(self, function):
        assert isinstance(function, Function)
        if function not in self.functions:
            self.functions.append(function)

    def add_func_atom(self, atom):
        assert isinstance(atom, Atom) and atom.is_function()
        function = atom.predicate
        self.add_function(function)
        for arg in atom.args:

            if not isinstance(arg, Parameter):
                self.add_object(arg)

    def add_perm_initialize(self, initialize):
        assert isinstance(initialize, Initialize)
        self.add_func_atom(initialize.atom)

        op = max if DEFAULT_COST == 0 else min
        self.perm_functions[initialize.atom] = op(self.perm_functions.get(
            initialize.atom, DEFAULT_COST), initialize.value)

    def add_temp_initialize(self, initialize):
        assert isinstance(initialize, Initialize)
        self.add_func_atom(initialize.atom)

        op = max if DEFAULT_COST == 0 else min
        self.temp_functions[initialize.atom] = op(self.temp_functions.get(
            initialize.atom, DEFAULT_COST), initialize.value)

    @property
    def perm_initialize(self):
        return {Initialize(atom, value) for atom, value in self.perm_functions.iteritems()}

    @property
    def temp_initialize(self):
        return {Initialize(atom, value) for atom, value in self.temp_functions.iteritems()}

    def add_formula(self, formula):
        for atom in formula.get_atoms():
            self.add_atom(atom)

    def add_action(self, operator):
        assert isinstance(operator, Action)
        if operator.name in self.name_to_action:
            raise ValueError('Repeated action name: %s' % operator.name)

        self.name_to_action[operator.name] = operator
        for atom in (operator.condition.get_atoms() | operator.effect.get_atoms()):
            if atom.is_function():
                self.add_func_atom(atom)
            else:
                self.add_atom(atom)
        for atom in operator.effect.get_atoms():
            if self.is_derived(atom):
                raise ValueError(
                    '%s is a derived and fluent predicate' % atom.predicate)
            if self.is_stream(atom):
                raise ValueError(
                    '%s is a stream and fluent predicate' % atom.predicate)
            if not atom.is_function():
                self.fluent_predicates.add(atom.predicate)

    def add_axiom(self, axiom):
        assert isinstance(axiom, Axiom)
        for atom in (axiom.condition.get_atoms() | axiom.effect.get_atoms()):
            self.add_atom(atom)
        if self.is_fluent(axiom.effect):
            raise ValueError('%s is a fluent and derived predicate' %
                             axiom.effect.predicate)
        if self.is_stream(axiom.effect):
            raise ValueError('%s is a stream and derived predicate' %
                             axiom.effect.predicate)
        if self.is_derived(axiom.effect):
            raise ValueError(
                'Derived predicate %s used in two axioms' % axiom.effect.predicate)
        self.derived_predicates[axiom.effect.predicate] = axiom
        self.axioms.append(axiom)

    def constants_pddl(self):
        s = ''
        for ty in self.type_to_objects:
            if len(self.type_to_objects[ty]) != 0:
                s += '\n\t' + ' '.join(obj.pddl()
                                       for obj in self.type_to_objects[ty])

                if ty is not None:
                    s += ' - ' + ty.pddl()
        return s

    def domain_pddl(self, costs, derived):
        s = '(define (domain %s)\n' % self.domain_name
        requirements = [':strips', ':typing', ':equality']
        if costs:
            requirements += [':action-costs']
        if derived:
            requirements += [':derived-predicates', ':adl']
        s += '(:requirements %s)\n' % ' '.join(requirements)

        s += '(:types ' + ' '.join(ty.pddl()
                                   for ty in self.type_to_objects if ty is not None) + ')\n'
        s += '(:constants' + self.constants_pddl() + ')\n'
        s += '(:predicates\n\t' + '\n\t'.join(predicate.pddl()
                                              for predicate in self.predicates.values()) + ')\n'
        if costs:

            s += '(:functions %s)\n' % '\n\t'.join(function.pddl()
                                                   for function in [TotalCost] + self.functions)
        if len(self.name_to_action) != 0:
            s += '\n' + '\n\n'.join(action.pddl(costs)
                                    for action in self.name_to_action.values()) + '\n'
        if len(self.axioms) != 0:
            s += '\n' + '\n\n'.join(axiom.pddl() for axiom in self.axioms)
        return s + ')\n'

    def get_initial_atoms(self):
        return self.initial_atoms | self.temporary_atoms | self.perm_initialize | self.temp_initialize

    def initial_fluents(self):
        return {atom for atom in self.get_initial_atoms() if isinstance(atom, Atom) and
                not self.is_stream(atom) and atom.predicate is not Concrete}

    def initial_stream_atoms(self):
        return {atom for atom in self.get_initial_atoms() if isinstance(atom, Atom) and self.is_stream(atom)}

    def initial_pddl(self, costs):
        atoms = list(self.get_initial_atoms())
        if costs:
            atoms.append(Initialize(TotalCost(), self.initial_cost))
        return '(' + '\n\t'.join([':init'] + [atom.pddl() for atom in atoms]) + ')'

    def goal_pddl(self):
        return '(:goal %s)' % self.goal_formula.pddl()

    def problem_pddl(self, costs):
        s = '(define (problem %s)\n' % self.problem_name
        s += '(:domain %s)\n' % self.domain_name

        s += self.initial_pddl(costs) + '\n'
        s += self.goal_pddl() + '\n'
        if costs:
            s += '(:metric minimize %s)' % TotalCost().pddl()
        return s + ')\n'

    def print_domain_statistics(self):
        print SEPARATOR
        print 'Types'
        for type in self.type_to_objects:
            print type, len(self.type_to_objects[type])

        predicate_map = defaultdict(list)
        for atom in self.initial_atoms:
            predicate_map[atom.predicate].append(atom)
        print
        print 'Predicates'
        for predicate in predicate_map:
            print predicate, len(predicate_map[predicate])

    def print_statistics(self):
        print SEPARATOR
        print 'Total time:', time() - self.start_time
        print 'Iterations:', self.iterations
        print 'Search time:', self.search_time
        print 'Average search time:', self.search_time / self.iterations
        print 'Empty:', self.empty
        print 'Intersect:', self.intersect
        print 'Resets:', self.resets
        print
        print 'Calls:', self.calls
        print 'Gen calls:', self.calls - self.test_calls
        print 'Test calls:', self.test_calls
        print 'Enumerated:', self.enumerated
        print
        print 'Problem statistics'
        for cs in self.problem.cond_streams:
            print cs, cs.calls, cs.call_time
