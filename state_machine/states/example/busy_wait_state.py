import rospy

from ..base._base_state import BaseState


class BusyWait(BaseState):
    """
    A simple state that infinitely busy-waits unless preempted.
    """

    def __init__(self, name, tree, outcome, conditions):
        super().__init__(name, tree, outcome, conditions)

    def behaviour(
        self,
    ):
        rospy.loginfo(
            "Running the BusyWait. \
            Expected behaviour is to infinitely wait unless pre-emptively killed."
        )
        while True:
            if self.preempt_requested():
                return
            rospy.sleep(1.0)
