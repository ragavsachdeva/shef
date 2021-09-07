import abc
from typing import List

import rospy
import smach
from state_machine.states.base._forced_transition_message import ForcedTransitionMessage

from ...utils.ledger import Ledger


class BaseTransitionState(smach.State):
    """
    The base state which all transition states inherit from.
    You shouldn't need to edit this file.
    Speak to @ragavsachdeva if you think otherwise.
    """

    def __init__(
        self,
        name: str,
        tree: str,
        child_states: List[str],
        conditions: List[List[str]],
        ledger: Ledger,
    ):
        smach.State.__init__(self, outcomes=child_states)
        self.name = name
        self._child_state_pointers = {}
        self.child_states = child_states
        self.tree = tree
        self.connected_monitor = None
        self.ledger = ledger
        self.data_store = {}
        self.forced_goto_state = None
        self.last_state = None
        self.current_state = None
        self.conditions = conditions

    @abc.abstractmethod
    def next_state(self) -> str:
        """
        Return the name of the name state you want to go to next.
        """
        pass

    def tear_down(self):
        """
        Add custom tear down functionality, every time the state machine
        associated with this transition state exits.
        Note: This does NOT get executed at every transition.
        """
        pass

    def force_goto(self, transition_message: ForcedTransitionMessage):
        """
        Preempts the currently active state and sets the forced_goto_state
        variable. Only intended to be called by the associated monitor.
        """
        # State doesn't exist in the state machine.
        if transition_message.target_state not in self.child_states:
            return
        if (
            transition_message.access_control == "ignore_if_in_given_states"
            and self.current_state in transition_message.potential_current_states
        ):
            return
        if (
            transition_message.access_control == "preempt_if_in_given_states"
            and self.current_state not in transition_message.potential_current_states
        ):
            return
        # Is there an active state?
        if self.current_state in self.child_states:
            self._child_state_pointers[self.current_state].request_preempt()
        self.forced_goto_state = transition_message.target_state

    def execute(self, userdata) -> str:
        """
        The main function that gets called by SMACH when the transition state
        is active. DO NOT call this function directly.
        """
        self.last_state = self.current_state
        self.current_state = None

        if self.last_state is None:
            self._initial_setup()

        if self.preempt_requested() or not self.connected_monitor.is_running:
            return self._preempt_and_transition()

        if self.forced_goto_state:
            next_state = self.forced_goto_state
            self.forced_goto_state = None
        else:
            next_state = self.next_state()

        self.current_state = next_state
        if next_state == self.child_states[-1]:
            self._wait_for_subscribers_to_be_released_and_transition(
                self.postexecution_subscribers, "EXIT"
            )
            self._clean_up()

        return next_state

    def _initial_setup(self):
        """
        Initial code that is run when the state machine is entered.
        """
        self.preexecution_subscribers = []
        self.postexecution_subscribers = []
        self._setup_subscribers()
        self._wait_for_subscribers_to_be_released_and_transition(
            self.preexecution_subscribers, "ENTER"
        )

    def _setup_subscribers(self):
        """
        Setup subscriptions to states that need to be waited for.
        """
        if self.conditions is None:
            return

        if "pre-execution" in self.conditions.keys():
            for state_tree, event_type in self.conditions["pre-execution"]:
                subscriber_id = self.ledger.subscribe_to_state(state_tree, event_type)
                self.preexecution_subscribers.append(subscriber_id)

        if "post-execution" in self.conditions.keys():
            for state_tree, event_type in self.conditions["post-execution"]:
                subscriber_id = self.ledger.subscribe_to_state(state_tree, event_type)
                self.postexecution_subscribers.append(subscriber_id)

    def _wait_for_subscribers_to_be_released_and_transition(
        self, subscriber_ids, transition_type
    ):
        """
        Busy waits for the subscibers to be released and
        atomically enters/exits the state.
        """
        successfully_transitioned = False
        while not successfully_transitioned:
            if self.preempt_requested():
                return self._preempt_and_transition(
                    force_exit=(transition_type == "EXIT")
                )
            successfully_transitioned = (
                self.ledger.atomically_check_subscriptions_and_trigger_event(
                    self.tree, subscriber_ids, transition_type
                )
            )
            rospy.sleep(0.1)

    def _preempt_and_transition(self, force_exit=True) -> str:
        """
        Exit the state machine.
        """
        self.service_preempt()
        if force_exit:
            self.ledger.force_exit_state(self.tree)
        next_state = self.child_states[-1]
        self._clean_up()
        return next_state

    def _clean_up(self):
        """
        Called exactly once - when exiting the state machine.
        """
        self.forced_goto_state = None
        self.last_state = None
        self.current_state = None
        self.tear_down()
        self.preexecution_subscribers = []
        self.postexecution_subscribers = []

    """
    The following functions are called when constructing
    and connecting state machines. Please do not call them.
    """

    def connect_child_state_pointers(self, child_states):
        for child_state in child_states:
            self._child_state_pointers[child_state.name] = child_state

    def connect_monitor(self, monitor):
        self.connected_monitor = monitor

    def get_outcome_pairs(self):
        return {child_state: child_state for child_state in self.child_states}

    def get_state_machine(self):
        return self
