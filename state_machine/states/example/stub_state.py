from ..base._base_state import BaseState


class StubState(BaseState):
    """
    A simple stub state that instantly exits.
    """

    def __init__(self, name, tree, outcome, conditions):
        super().__init__(name, tree, outcome, conditions)

    def behaviour(
        self,
    ):
        pass
