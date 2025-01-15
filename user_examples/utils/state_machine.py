# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Hierarchical state machine tools for representing behaviors.
#
# Basic state interface is given by State, and HierarchicalState provides an interface to building
# up more complicated hierarchies of state machines. run_state_machine() and RunState provide tools
# for running state machines using the HierarchicalState tools.

import time
from .behavior_helpers import go_home


class State(object):
    """ Basic state interface.

    Transitions are checked first each cycle and are handled in a single cycle. exit() is called on
    the previous state, and enter() is called on the new state (in that order). step() is then
    called on the new state.

    See run_state_machine() / RunState.run() for the specifics of how states are processed.
    """

    def __init__(self):
        pass

    def enter(self):
        pass

    def step(self):
        pass

    def exit(self):
        pass

    def transition(self):
        return self


class HierarchicalState(State):
    """ A hierarchical state encapculating a full state machine internally.

    enter() resets the active state to the initial state and calls enter on it.
    step() steps the underlying state machine, performing any transitions necessary. See below for a
      description of state transition behavior.
    exit() if there is an active state, calls exit on that active state.
    transition() stays in the current (hierarchical) state while there is an active state, otherwise
      returns None.

    Checks transitions at the beginning of each step() call, and calls step() on the current active
    state at the end of each step() call. If there is a transition, calls exit() on the previous
    state, and calls enter() on the next state() and resets the active state to that new state so
    it's step() function will be called at the end of the call.

    RunState (and run_state_machine) provides a simple interface to running a state machine using
    this HierarchicalState as its internal implementation.
    """

    def __init__(self, init_state):
        self.init_state = init_state
        self.active_state = None
        self.transitions = []

    def enter(self):
        if self.init_state:
            self.active_state = self.init_state
            self.active_state.enter()

    def step(self):
        """ Step the underlying state machine until it can no longer transition.
        """
        if self.active_state is None:
            return

        next_state = self.active_state.transition()
        if next_state is None:
            self.active_state.exit()
            self.active_state = None
            return  # done

        if next_state != self.active_state:
            self.active_state.exit()
            next_state.enter()
            self.active_state = next_state

        self.active_state.step()

    def exit(self):
        if self.active_state is not None:
            self.active_state.exit()

    def transition(self):
        for trans in self.transitions:
            next_state = trans()
            if next_state is not None:
                return next_state

        if self.active_state is None:
            return None
        return self

    def is_active(self):
        return self.active_state is not None


class MultiHierarchicalState(State):
    """ A multi-hierarchical state machine is a state machine which runs simultaneously multiple
    internal state machines. Does not do multithreading, but enters, steps, and exits each of the
    internal machines in lock step in the order they're provided on construction.
    """

    def __init__(self, init_states):
        """ Construct to run the provided state machines give by their initial states.
        """
        self.internal_machines = [HierarchicalState(init_state=state) for state in init_states]

    def enter(self):
        """ Enter into each of the internal state machines.
        """
        for machine in self.internal_machines:
            machine.enter()

    def step(self):
        """ Step each of the internal state machines.
        """
        for machine in self.internal_machines:
            machine.step()

    def transition(self):
        """ Transition to self and keep running as long as any of the internal machines are still
        active.
        """
        for machine in self.internal_machines:
            if machine.is_active():
                return self
        return None

    def exit(self):
        """ Exit from each of the internal state machines.
        """
        for machine in self.internal_machines:
            machine.exit()


class RunState(HierarchicalState):
    """ Creates a hierarchical state with just a single submachine, and provides a run method to run
    that machine until completion.
    """

    def __init__(self, start_state, rate, domains=None):
        self.start_state = start_state
        self.rate = rate
        self.domains = domains
        super().__init__(self.start_state)

    def run(self):
        """ Steps the underlying state machine until there's no longer an active state.

        Steps at the specified rate.
        """
        self.enter()
        try:
            while True:

                if any(x.stop is True for x in self.domains):
                    for domain in self.domains:
                        go_home(domain.franka)
                    print("State machine terminated")
                    return
                if self.active_state is None:
                    for domain in self.domains:
                        go_home(domain.franka)
                    return
                self.step()
                time.sleep(self.rate)
        except KeyboardInterrupt:
            pass


def run_state_machine(state, rate, domain=None):
    """ Convenience method for running a state machine. Equivalent to RunState(state, rate).run().
    """
    RunState(state, rate, domain).run()


class Behavior(HierarchicalState):
    """ A behavior is a state machine that's run to completion. At the end of the behavior, a
    provided terminal_transition() method is called to give the next state. If there is no provided
    terminal_transition method, returns None on termination.

    If behavior_machine is None, it will immediately transition away following the expected behavior
    of the hierarchical state machine
    """

    def __init__(self, behavior_machine=None, terminal_transition=None):
        self.behavior_machine = behavior_machine
        self.terminal_transition = terminal_transition
        self.on_enter = None
        self.on_exit = None
        super().__init__(init_state=self.behavior_machine)

    def transition(self):
        """ Transition based on the underlying behavior state machine until that behavior terminates
        as indicated by the transition returning None. Then transition based on the
        terminal_transition() method provided on construction if one exists, otherwise returns None.
        """
        next_state = super().transition()
        if next_state is not None:
            return next_state

        if self.terminal_transition is None:
            return None
        return self.terminal_transition()

    def enter(self):
        if self.on_enter is not None:
            self.on_enter()
        super().enter()

    def exit(self):
        if self.on_exit is not None:
            self.on_exit()
        super().exit()


class NextStateTransition(object):
    """ A simple terminal transition function (object) representing a transition to a pre-specified
    next state.
    """

    def __init__(self, next_state):
        self.next_state = next_state

    def __call__(self):
        return self.next_state


class ThinkAndRun(HierarchicalState):
    """ Think and run state machines.

    Enables running multiple "thinking" state machines simultaneously preceeding the running of an
    actual robot execution state machine.
    """

    def __init__(self, think_machines, run_machine, max_time=5):
        """ Create the think and run state machine from a collection of think machines and a run
        machine. Each think machine and run machine should be an initial state which will be entered
        into to start the behavior. On enter(), each of the think machine initial states are entered
        into. Those are all stepped until there are no more running think behaviors. One those think
        behaviors are finished, the provided init state of the run machine is entered into. That
        machine is run to completion before exiting.
        """
        self.think_behavior = Behavior(MultiHierarchicalState(think_machines))
        self.run_behavior = Behavior(run_machine)

        self.think_behavior.terminal_transition = NextStateTransition(self.run_behavior)
        self.domain = run_machine.domain
        self.think_time = max_time
        super().__init__(self.think_behavior)

    def enter(self):
        self.start_time = self.domain.time
        super().__init__(self.think_behavior)
        super().enter()

    def transition(self):
        if self.think_time is not None and (self.domain.time - self.start_time) < self.think_time:
            return self
        super().__init__(self.run_behavior)
        return self.run_behavior
