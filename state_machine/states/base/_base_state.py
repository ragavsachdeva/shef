import abc
from typing import List

import rospy
import smach
from constants.configs import get_spawned_robot_names
from easydict import EasyDict
from loguru import logger

from ...utils.ledger import Ledger


class BaseState(smach.State):
    """
    The base state which all our custom states should inherit from.
    Note: It does not have a monitor.
    You shouldn't need to edit this file.
    Speak to @ragavsachdeva if you think otherwise.
    """

    def __init__(self, name: str, tree: str, outcome: str, conditions: List[List[str]]):
        smach.State.__init__(self, outcomes=[outcome])
        self.outcome = outcome
        self.name = name
        self.tree = tree
        self.data_store = None
        self.ledger = None
        self.conditions = conditions

    @abc.abstractmethod
    def behaviour(self):
        """
        Intended behaviour of this state lives here.
        This is the main function that needs to be implemented.
        """
        pass

    def tear_down(self):
        """
        Add functionality to reset the parameters when this base state exits.
        """
        pass

    def execute(self, userdata) -> str:
        """
        The main function that gets called by SMACH when the state
        is active. DO NOT call this function directly.
        """
        self._initial_setup()
        self.behaviour()
        if self.preempt_requested():
            return self._preempt_and_transition()
        self._wait_for_subscribers_to_be_released_and_transition(
            self.postexecution_subscribers, "EXIT"
        )
        self._clean_up()
        return self.outcome

    def _initial_setup(self):
        """
        Initial code that is run when the base state is entered.
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
        Busy waits for the subscibers to be released
        and atomically enters/exits the state.
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
        self._clean_up()
        if self.preempt_requested():
            logger.warning(
                f"WARNING: Possible Race Condition.\
                \nPreempt was serviced but it got reset in {self.tree}"
            )
        return self.outcome

    def _clean_up(self):
        """
        Called when exiting the base state.
        """
        self.tear_down()
        self.preexecution_subscribers = []
        self.postexecution_subscribers = []

    """
    The following functions are called when constructing
    and connecting state machines. Please do not call them.
    """

    def add_states(self, states):
        pass

    def get_state_machine(self):
        return self

    def connect_data_store(self, data_store: EasyDict):
        self.data_store = data_store

    def connect_ledger(self, ledger: Ledger):
        self.ledger = ledger
