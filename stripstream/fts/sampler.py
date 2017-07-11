from stripstream.pddl.cond_streams import ExternalGenCondStream
from stripstream.fts.variable import FreeParameter
from stripstream.fts.constraint import Constraint, convert_constraint


class Sampler(object):

    def __init__(self, values, gen, inputs=(), domain=(), name=None, **kwargs):
        self.cons = filter(lambda c: isinstance(c, Constraint), values)
        self.gen = gen
        self.inputs = tuple(inputs)
        self.domain = domain

        self.name = name if name is not None else self.__class__.__name__ + \
            str(values)
        self.kwargs = kwargs
        outputs = []
        for val in values:
            if isinstance(val, Constraint):
                for par in val.values:
                    assert isinstance(par, FreeParameter)
                    if par not in self.inputs and par not in outputs:
                        outputs.append(par)
            elif isinstance(val, FreeParameter):
                if val not in self.inputs and val not in outputs:
                    outputs.append(val)
            else:
                raise ValueError('Unexpected value %s' % val)
        self.outputs = tuple(outputs)

    def __repr__(self):
        return self.name


def convert_sampler(sampler):
    def generator(inputs):
        input_values = tuple(const.type.get_value(const) for const in inputs)
        for output_list in sampler.gen(*input_values):
            assert type(output_list) == list
            new_output_list = []
            for outputs in output_list:
                if type(outputs) not in [list, tuple]:
                    outputs = [outputs]
                assert len(outputs) == len(sampler.outputs)
                new_output_list.append(tuple(par.type(out)
                                             for par, out in zip(sampler.outputs, outputs)))
            yield new_output_list

    return ExternalGenCondStream(
        generator=generator,
        name=sampler.name,
        inputs=sampler.inputs,
        outputs=sampler.outputs,
        conditions=map(convert_constraint, sampler.domain),
        effects=map(convert_constraint, sampler.cons),
        **sampler.kwargs
    )
