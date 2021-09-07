from .__base_state_machine import BaseStateMachine
from .__smach_concurrence_container import SmachConcurrenceContainer


class ConcurrentStateMachine(BaseStateMachine):
    """
    A concurrent state machine that can be pre-empted using its monitor.
    It's a container that can simultaneously run multiple states.
    You shouldn't need to edit this file.
    Speak to @ragavsachdeva if you think otherwise.
    """

    def __init__(self, name: str, tree: str, outcome: str):
        super().__init__(
            name=name, tree=tree, outcomes=[outcome], default_outcome=outcome
        )
        self.outcome = outcome
        self.type = "ConcurrentStateMachine"

    def add_states(self, states):
        with self.state_machine:
            for state in states:
                SmachConcurrenceContainer.add(state.name, state.get_state_machine())

    def outcome_callback(self, outcome_map):
        return self.outcome
