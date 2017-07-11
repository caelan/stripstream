from stripstream.fts.constraint import Eq


class FTSProblem(object):

    def __init__(self, state_vars, control_vars, transition, samplers, initial_state, goal_constraints):
        self.state_vars = state_vars
        self.control_vars = control_vars
        self.transition = transition
        self.samplers = samplers
        self.initial_state = initial_state
        self.goal_constraints = goal_constraints

    def get_variable_map(self):
        return {var.name: var for var in self.state_vars + self.control_vars}

    def get_constraint_forms(self):
        con_forms = set()
        for con in self.initial_state + self.goal_constraints:
            if con.constraint != Eq:
                con_forms.add(con.constraint)
        for clause in self.transition:
            for con in clause.constraints:
                if con.constraint != Eq:
                    con_forms.add(con.constraint)
        for sampler in self.samplers:
            for con in list(sampler.cons) + list(sampler.domain):
                con_forms.add(con.constraint)
        return con_forms

    def __repr__(self):
        return 'Factored Transition System Problem\n'           'State Variables: %s\n'           'Control Variables: %s\n'           'Transition Relation: %s\n'           'Samplers: %s\n'           'Initial State: %s\n'           'Goal Constraints: %s' % (self.state_vars, self.control_vars,
                                                                                                                                                                                                                                                                 self.transition, self.samplers, self.initial_state, self.goal_constraints)
