import rospy

from ..base._base_state import BaseState


class DummyState(BaseState):
    """
    A simple dummy state that exits after some time.
    """

    def __init__(self, name, tree, outcome, conditions):
        super().__init__(name, tree, outcome, conditions)

    def behaviour(self):
        waiting_time = 5
        rospy.loginfo(
            f"Running {self.name} state. "
            + f"Expected behaviour is to sleep for {waiting_time} second."
        )
        for x in range(waiting_time):
            if self.preempt_requested():
                rospy.loginfo(f"Pre-empting {self.name} state.")
                return
            rospy.sleep(1.0)

        rospy.loginfo(f"Exiting {self.name} state.")
