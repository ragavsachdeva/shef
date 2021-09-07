import abc
from typing import List

from .__smach_concurrence_container import SmachConcurrenceContainer
from .__smach_monitor_state import SmachMonitorState


class BaseStateMachine:
    """
    The base state machine that all our custom state machines inherit from.
    You shouldn't need to directly inherit it.
    Speak to @ragavsachdeva if you think otherwise.
    """

    def __init__(self, name: str, tree: str, outcomes: List[str], default_outcome: str):
        self.state_machine = SmachConcurrenceContainer(
            outcomes=outcomes,
            default_outcome=default_outcome,
            outcome_cb=self.outcome_callback,
            child_termination_cb=self.child_termination_callback,
        )
        self.tree = tree
        self.outcome = outcomes[0]
        self.name = name
        self.monitor_name = f"{self.tree}_monitor"
        self.monitor = SmachMonitorState(self.monitor_name)
        with self.state_machine:
            SmachConcurrenceContainer.add(self.monitor_name, self.monitor)

    # todo(ragavsachdeva): inspect this function.
    @abc.abstractmethod
    def outcome_callback(self, outcome_map):
        """
        Executed when all the states have terminated.
        """
        pass

    def child_termination_callback(self, outcome_map):
        """
        Called every time a child state terminates (including the monitor).
        Return True if you want to stop the BaseStateMachine.
        """
        if outcome_map[self.monitor_name] == "invalid":
            return True

        all_children_terminated = True
        for state, outcome in outcome_map.items():
            if state == self.monitor_name:
                continue
            if outcome is None:
                all_children_terminated = False

        return all_children_terminated

    """
    The following functions are called when constructing
    and connecting state machines. Please do not call them.
    """

    def get_state_machine(self):
        return self.state_machine

    def request_preempt(self):
        self.state_machine.request_preempt()

    def execute(self):
        self.state_machine.execute()
