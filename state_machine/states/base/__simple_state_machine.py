from .__base_state_machine import BaseStateMachine
from .__smach_concurrence_container import SmachConcurrenceContainer
from .__smach_state_machine import SmachStateMachine


class SimpleStateMachine(BaseStateMachine):
    """
    A simple state machine that can be pre-empted using its monitor.
    Can only run a single child state at a time.
    You shouldn't need to edit this file.
    Speak to @ragavsachdeva if you think otherwise.
    """

    def __init__(self, name: str, tree: str, outcome: str):
        super().__init__(
            name=name, tree=tree, outcomes=[outcome], default_outcome=outcome
        )
        self.type = "SimpleStateMachine"
        self.inner_state_machine = SmachStateMachine(outcomes=[outcome])
        self.outcome = outcome
        with self.state_machine:
            SmachConcurrenceContainer.add(
                "inner_state_machine", self.inner_state_machine
            )

    def add_states(self, states, transition_state):
        transition_state.connect_monitor(self.monitor)
        transition_state.connect_child_state_pointers(states)
        self.monitor.connect_transition_state(transition_state)
        with self.inner_state_machine:
            SmachStateMachine.add(
                "transition",
                transition_state.get_state_machine(),
                transition_state.get_outcome_pairs(),
            )
            for state in states:
                SmachStateMachine.add(
                    state.name,
                    state.get_state_machine(),
                    transitions={state.outcome: "transition"},
                )

    def outcome_callback(self, outcome_map) -> str:
        return self.outcome
