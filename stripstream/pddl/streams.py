from time import time


class Stream(object):
    """
    Stream abstract class.
    """

    def __init__(self, cond_stream, inputs):
        """
        :param cond_stream: a :class:`.ConditionalStream` which produced this stream
        :param inputs: a list of values for parameters ``cond_stream.inputs`` that are the inputs to this stream
        """
        self.cond_stream = cond_stream
        self.inputs = tuple(inputs)
        self.history = {}
        self.enumerated = False
        self.call_times = []
        self.call_history = []
        self.reset()
        self.conditions = self.cond_stream.instantiate_conditions(self.inputs)

    def instantiate_conditions(self):
        return self.conditions

    def instantiate_effects(self, outputs):
        return self.cond_stream.instantiate_effects(self.inputs, outputs)

    def get_values(self, **kwargs):
        """
        :returns: a list of :class:`.Constant` and :class:`.Atom` of the values produced by the stream
        """
        raise NotImplementedError()

    @property
    def call_time(self):
        return sum(self.call_times)

    @property
    def calls(self):
        return len(self.call_times)

    def call(self, **kwargs):
        assert not self.enumerated
        if not self.called and self.calls:
            self.called = True
            self.enumerated = self.true_enumerated
            all_values = []
            for values in self.call_history:
                all_values += list(values)
            return all_values
        t0 = time()
        self.called = True
        new_values = tuple(self.get_values(**kwargs))
        for value in new_values:
            if value not in self.history:
                self.history[value] = []
            if len(self.history) not in self.history[value]:
                self.history[value].append(len(self.history))
        self.call_history.append(new_values)
        self.call_times.append(time() - t0)
        return new_values

    def reset(self):
        self.called = False
        self.true_enumerated = self.enumerated
        self.enumerated = False

    def __repr__(self):
        return '%s(%s | %s)' % (self.cond_stream.__class__.__name__, self.cond_stream.outputs, self.inputs)
    __str__ = __repr__


class StrictStream(Stream):
    """
    Stream abstract class which only supports returning tuples of values which satisfy ``self.cond_stream.outputs``.
    """

    def get_next(self, **kwargs):
        """
        :returns: a list of tuples of :class:`.Constant` which satisfy ``self.cond_stream.outputs``
        """
        raise NotImplementedError()

    def get_values(self, **kwargs):
        assert not self.enumerated
        values = []
        for outputs in self.get_next(**kwargs):
            assert len(self.cond_stream.outputs) == len(outputs) and all(
                p.type == a.type for p, a in zip(self.cond_stream.outputs, outputs))
            values += list(outputs) + \
                self.cond_stream.instantiate_effects(self.inputs, outputs)
        return values


class GeneratorStream(StrictStream):
    """
    :class:`.StrictStream` which can be specified using a function to a generator.
    """

    def __init__(self, cond_stream, inputs):
        self.generator = self.get_generator(inputs)
        super(GeneratorStream, self).__init__(cond_stream, inputs)

    def get_generator(self, inputs):
        """
        Generator function which must be overridden.

        :param inputs: a list of values for parameters ``cond_stream.inputs`` that are the inputs to the stream
        :returns: a generator of tuples of :class:`.Constant` which satisfy ``self.cond_stream.outputs``
        """
        raise NotImplementedError()

    def get_next(self, **kwargs):
        try:
            values = next(self.generator)
            if not isinstance(values, list):
                return [values]
            return values
        except StopIteration:
            self.enumerated = True
            return []


class ListStream(StrictStream):
    """
    :class:`.StrictStream` which can be specified using a function to a list.
    """

    def get_list(self, inputs):
        """
        List function which must be overridden.

        :param inputs: a list of values for parameters ``cond_stream.inputs`` that are the inputs to the stream
        :returns: a list of tuples of :class:`.Constant` which satisfy ``self.cond_stream.outputs``
        """
        raise NotImplementedError()

    def get_next(self, **kwargs):
        self.enumerated = True
        return self.get_list(self.inputs)


class FunctionStream(StrictStream):
    """
    :class:`.StrictStream` which can be specified using a function.
    """

    def function(self, inputs):
        """
        Function which must be overridden.

        :param inputs: a list of values for parameters ``cond_stream.inputs`` that are the inputs to the stream
        :returns: a tuple of :class:`.Constant` which satisfies ``self.cond_stream.outputs``
        """
        raise NotImplementedError()

    def test(self, inputs):
        """
        Test which may optionally be overridden.

        :param inputs: a list of values for parameters ``cond_stream.inputs`` that are the inputs to the stream
        :returns: a boolean value ``{False, True}``
        """
        return True

    def get_next(self, **kwargs):
        self.enumerated = True
        if self.test(self.inputs):
            return [self.function(self.inputs)]
        return []


class TestStream(StrictStream):
    """
    :class:`.StrictStream` which can be specified using a test.
    """

    def test(self, inputs):
        """
        Test which must be overridden.

        :param inputs: a list of values for parameters ``cond_stream.inputs`` that are the inputs to the stream
        :returns: a boolean value ``{False, True}``
        """
        raise NotImplementedError()

    def get_next(self, **kwargs):
        self.enumerated = True
        return [()] if self.test(self.inputs) else []
