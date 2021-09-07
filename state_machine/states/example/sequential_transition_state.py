from typing import List

from ...utils.ledger import Ledger
from ..base._base_transition_state import BaseTransitionState


class SequentialTransitionState(BaseTransitionState):
    """
    Transition state to sequentially execute all the child states.
    """

    def __init__(
        self,
        name: str,
        tree: str,
        outcomes: List[str],
        conditions: List[List[str]],
        ledger: Ledger,
    ):
        super().__init__(name, tree, outcomes, conditions, ledger)

    def next_state(
        self,
    ):
        index = 0
        if self.last_state is not None:
            index = self.child_states.index(self.last_state) + 1
        return self.child_states[index]
