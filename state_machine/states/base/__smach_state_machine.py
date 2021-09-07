from typing import List

import smach


class SmachStateMachine(smach.StateMachine):
    """
    A wrapper around SMACH State Machine.
    You shouldn't need to edit this file.
    Speak to @ragavsachdeva if you think otherwise.
    """

    def __init__(self, outcomes: List[str]):
        super().__init__(outcomes=outcomes)
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
