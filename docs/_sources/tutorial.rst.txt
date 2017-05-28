tutorials
=========

scripts.run_tutorial module
---------------------------

This tutorial considers a 1D task and motion planning domain with 1 robot and 3 blocks.
Robot configurations and block poses are nonnegative integers from 0 to ``num_poses``.
The robot can move to other configurations, pick blocks, and place blocks.
For simplicity, assume that the robot cannot collide with blocks when it moves, picks, or places.
Thus, the only collisions are between pairs of blocks.

We consider the following problem specification.
The robot starts at configuration 0.
Each block i starts at pose i.
The goal conditions require each block i be shifted right one pose to pose i+1.
There are ``num_poses = 10^10`` possible poses for each block.

.. code:: python

  blocks = ['block%i'%i for i in range(3)]
  num_poses = pow(10, 10) # 10^10 possible poses!

  initial_config = 0 # the initial robot configuration is 0
  initial_poses = {block: i for i, block in enumerate(blocks)} # the initial pose for block i is i

  goal_poses = {block: i+1 for i, block in enumerate(blocks)} # the goal pose for block i is i+1

We start by creating the object types for configurations (``CONF``), blocks (``BLOCK``), and poses (``POSE``).
Types are used to restrict the domain of predicates to only apply to sensible objects.
These can be alternatively thought of as implicit unary predicates.

.. code:: python

    CONF, BLOCK, POSE = Type(), Type(), Type()

Next, we define the predicates (``Pred``) we will use.
A predicate is specified by a list of types for its arguments.
For our domain, we use the fluent predicates ``AtConf`` to model the robot's configuration,
``AtPose`` to model the pose of each block, ``HandEmpty`` to indicate the robot is not holding anything, and
``Holding`` to specify the block the robot is holding.

We will later define an axiom that uses the derived predicate ``Safe`` to compactly handle collision checking.
``Safe`` indicates that the first block is at some pose that is not in collision with the second block at the specified pose.

Finally, we define two static predicates, predicates that have constant truth value throughout a problem.
``LegalKin`` indicates that a block pose and robot configuration satisfy a kinematic constraint.
Namely, the robot must be at the same pose as the block in order to pick or place the block.
``CollisionFree`` is true if and only if the first block at the first pose is not in collision with the second block
at the second pose.

.. code:: python

  # Fluent predicates
  AtConf = Pred(CONF)
  AtPose = Pred(BLOCK, POSE)
  HandEmpty = Pred()
  Holding = Pred(BLOCK)

  # Derived predicates
  Safe = Pred(BLOCK, BLOCK, POSE)

  # Static predicates
  LegalKin = Pred(POSE, CONF)
  CollisionFree = Pred(BLOCK, POSE, BLOCK, POSE)

We define several parameters (:class:`.Param`) that will be arguments of several actions, axioms, and conditional streams.
Each parameter is of the type specified by its argument and had a unique, anonymous name.
Parameters are only meaningful within the local scope of a action, axiom, or conditional stream.
Sharing parameters simply results in more compact code.

.. code:: python

  B1, B2 = Param(BLOCK), Param(BLOCK)
  P1, P2 = Param(POSE), Param(POSE)
  Q1, Q2 = Param(CONF), Param(CONF)

Next, we identify three actions (:class:`.Action`). Each action is specified by a ``name``, list of ``parameters``,
``condition`` logical formula, and ``effect`` logical formula. Logical formulas are nested combinations of
:class:`.Atom`, :class:`.Not`, :class:`.And`, :class:`.Or`, :class:`.Exists`, :class:`.ForAll`, :class:`.Equal`, and :class:`.When`.

The ``'pick'`` action has a block ``B1``, pose ``P1``, and configuration ``Q1`` parameter. Its condition is that
block ``B1`` be at pose ``P1``, the robot's hand is empty, the robot be at configuration ``Q1``, and
pose ``P1`` and configuration ``Q1`` satisfy the kinematic constraint.
Its effect is that the robot is now holding ``B1``, block ``B1`` is no longer at pose ``P1``, and
the robot's hand is no longer empty.

The ``'place'`` action is the inverse of a the ``'pick'`` action. It has an additional condition that each of the
other blocks ``B2`` is at a safe pose with respect to block ``B1`` at pose ``P1``.

The ``'move'`` action has parameters ``Q1`` and ``Q2`` for the start and end configurations. Its condition is that
the robot starts at configuration ``Q1`` and its effect is that the robot is at configuration ``Q2`` instead of configuration ``Q1``.

.. code:: python

 actions = [
    Action(name='pick', parameters=[B1, P1, Q1],
      condition=And(AtPose(B1, P1), HandEmpty(), AtConf(Q1), LegalKin(P1, Q1)),
      effect=And(Holding(B1), Not(AtPose(B1, P1)), Not(HandEmpty()))),

    Action(name='place', parameters=[B1, P1, Q1],
      condition=And(Holding(B1), AtConf(Q1), LegalKin(P1, Q1),
        ForAll([B2], Or(Equal(B1, B2), Safe(B2, B1, P1)))), # TODO - convert to finite blocks case?
      effect=And(AtPose(B1, P1), HandEmpty(), Not(Holding(B1)))),

    Action(name='move', parameters=[Q1, Q2],
      condition=AtConf(Q1),
      effect=And(AtConf(Q2), Not(AtConf(Q1)))),
  ]

We define an axiom (:class:`.Axiom`) that automatically infers ``Safe`` at each state before an action is performed.
``Safe`` is true if ``B2`` at its current pose ``P2`` is not in collision with ``B1`` at pose ``P1``.

As an aside, one could avoid using this axiom by substituting ``Safe`` for ``condition`` within the ``'place'`` action.
However, this would greatly increase the number of implicit parameters for ``'place'`` as it would then
involve a pose parameter for each object. Thus, the number of instantiations would grow exponentially in the
number of blocks which is undesirable for practical performance.
Axioms allow us to avoid exponential growth by creating an intermediate rule which infers safety for each block ``B2`` separately.

.. code:: python

  axioms = [
    Axiom(effect=Safe(B2, B1, P1),
          condition=Exists([P2], And(AtPose(B2, P2), CollisionFree(B1, P1, B2, P2)))), # Infers B2 is at a safe pose wrt B1 at P1
  ]

In order to represent the set of poses as well as evaluate ``LegalKin`` and ``CollisionFree``, we define several conditional streams.
A :class:`.GeneratorStream` is given by a list of parameter ``inputs``, parameter ``outputs``, conjunctive ``conditions``
that ``input`` values must satisfy, conjunctive ``effects`` which ``inputs + outputs`` will satisfy, and a function ``generator``
from values of ``inputs`` to a generator producing values of ``outputs``.

The first :class:`.GeneratorStream` enumerates the set of poses from 0 to ``num_poses-1``. It has no ``inputs`` and therefore
no ``conditions``, a single pose output ``P1``, but no ``effects``. Its ``generator simply enumerates poses ``0, 1, ..., num_poses``.

The second :class:`.GeneratorStream` enumerates the configuration kinematic solutions for a given pose.
It has a single pose input ``P1`` and a single configuration output ``Q1``.
``P1`` and ``Q1`` are guarantee to satisfy ``LegalKin`` as indicated using ``outputs``.
Its ``generator`` just returns ``p`` as there is only one possible configuration per pose.

A :class:`.TestStream` is a special type of :class:`.GeneratorStream` which has no ``outputs``.
Thus, it can be specified using a boolean ``test`` function which determines if ``effects`` hold instead of a ``generator``.

The :class:`.TestStream` evaluates collision checks between block ``B1`` at pose ``P1`` and block ``B2`` at pose ``P2``.
In our simplified model, two blocks are not in collision if they are at different poses.

.. code:: python

  # Conditional stream declarations
  cond_streams = [
    GeneratorStream(inputs=[], outputs=[P1], conditions=[], effects=[],
                    generator=lambda: xrange(num_poses)), # Enumerating all poses

    GeneratorStream(inputs=[P1], outputs=[Q1], conditions=[], effects=[LegalKin(P1, Q1)],
                    generator=lambda p: [p]), # Inverse kinematics

    TestStream(inputs=[B1, P1, B2, P2], conditions=[], effects=[CollisionFree(B1, P1, B2, P2)],
               test=lambda b1, p1, b2, p2: p1 != p2, eager=True), # Collision checking
  ]

The initial state is represented as a list of ``initial_atoms``.
For our problem, the robot starts at ``initial_config`` with its hand empty.
Each block starts at its corresponding initial pose.

.. code:: python

  initial_atoms = [
    AtConf(initial_config),
    HandEmpty()
  ] + [
    AtPose(block, pose) for block, pose in initial_poses.iteritems()
  ]

The set of goal states is represented as a list of ``goal_literals``.
For our problem, each block has a goal condition that it ends up at its corresponding goal pose.

.. code:: python

  goal_literals = [AtPose(block, pose) for block, pose in goal_poses.iteritems()]

STRIPStream automatically infers the type of and stores any objects passed to predicates.
In the event in which an object is not already included in the ``initial_atoms``, ``goal_literals``, ``actions``, or ``axioms``,
it can be specified using ``constants``. Because STRIPStream cannot infer the type of a standalone object, it must be
wrapped with the desired type. Suppose that the robot can move off the line to a ``None`` configuration. We can
indicate this by including ``CONF(None)`` in ``constants``.

.. code:: python

  constants = [
    CONF(None) # Any additional objects
  ]

Putting this all together, we arrive at the definition of a :class:`.STRIPStreamProblem` instance.

.. code:: python

  problem = STRIPStreamProblem(initial_atoms, goal_literals, actions + axioms, cond_streams, constants)

Finally, we can solve this problem by calling :func:`.incremental_planner` or :func:`.focused_planner`.
Each planner has an argument ``search`` which specifies a search subroutine to use within the planner.
This can currently be either :func:`.get_bfs` to use an inefficient Python breadth-first search
or :func:`.get_fast_downward` to use an efficient PDDL planner.
However, one must install FastDownward to use :func:`.get_fast_downward`.

.. code:: python

  #search = get_fast_downward('eager') # dijkstra | astar | wastar1 | wastar2 | wastar3 | eager | lazy
  search = get_bfs()

  plan, _ = incremental_planner(problem, search=search, frequency=1)
  #plan, _ = focused_planner(problem, search=search, greedy=False)
  print
  print 'Plan:', convert_plan(plan)

You can run this tutorial using the following command.

.. code::

    python -m scripts.run_tutorial

.. automodule:: scripts.run_tutorial
    :members: scripts.run_tutorial.create_problem
    :show-inheritance:
