import itertools
from typing import List

from ...utils.ledger import Ledger
from ..base._base_transition_state import BaseTransitionState


class LoopTransitionState(BaseTransitionState):
    """
    Transition state to sequentially execute all the child states and
    then loop around. The only way to exit the state machine would be to
    preemptively exit.
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
        self._state_sequence = self._get_state_sequence()

    def next_state(self) -> str:
        return next(self._state_sequence)

    def tear_down(self):
        self._state_sequence = self._get_state_sequence()

    def _get_state_sequence(self):
        return itertools.cycle(self.child_states[:-1])


class LoopXTimesTransitionState(BaseTransitionState):
    """
    Transition state to sequentially execute all the child states and
    then loop arounds X times.
    """

    def __init__(
        self,
        name: str,
        tree: str,
        outcomes: List[str],
        conditions: List[List[str]],
        ledger: Ledger,
        x: int = 1,
    ):
        super().__init__(name, tree, outcomes, conditions, ledger)
        self._num_iterations = x
        self._state_sequence = self._get_state_sequence()

    def next_state(self) -> str:
        return next(self._state_sequence)

    def tear_down(self):
        self._state_sequence = self._get_state_sequence()

    def _get_state_sequence(self):
        main_states = self.child_states[:-1]
        terminal_state = self.child_states[-1]
        # Repeat the main child states x times and then chain into a single iterator:
        state_sequence = itertools.chain.from_iterable(
            itertools.repeat(main_states, self._num_iterations)
        )
        # Append the terminal state:
        return itertools.chain(state_sequence, (terminal_state,))
