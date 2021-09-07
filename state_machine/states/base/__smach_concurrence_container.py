from typing import Callable, List

import smach


class SmachConcurrenceContainer(smach.Concurrence):
    """
    A wrapper around SMACH Concurrent State Machine.
    You shouldn't need to edit this file.
    Speak to @ragavsachdeva if you think otherwise.
    """

    def __init__(
        self,
        outcomes: List[str],
        default_outcome: str,
        outcome_cb: Callable,
        child_termination_cb: Callable,
    ):
        super().__init__(
            outcomes=outcomes,
            default_outcome=default_outcome,
            outcome_cb=outcome_cb,
            child_termination_cb=child_termination_cb,
        )
        self.register_start_cb(start_cb=self.start_callback, cb_args=[])
        self.register_termination_cb(
            termination_cb=self.termination_callback, cb_args=[]
        )

    def start_callback(self, *args):
        pass

    def termination_callback(self, *args):
        pass

    def get_state_machine(self):
        return self
