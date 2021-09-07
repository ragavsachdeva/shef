import smach_ros
from state_machine.states.base._forced_transition_message import ForcedTransitionMessage
from std_msgs.msg import String


class SmachMonitorState(smach_ros.MonitorState):
    """
    A wrapper around SMACH Monitor State.
    You shouldn't need to edit this file.
    Speak to @ragavsachdeva if you think otherwise.
    """

    def __init__(self, topic: str):
        super().__init__(topic=topic, msg_type=String, cond_cb=self.monitor_callback)
        self.is_running = True
        self.transition_state = None

    def monitor_callback(self, userdata, message: str):
        """
        Gets called every time a message is published to the monitor channel.
        Returning False will stop this entire BaseStateMachine.
        Returning True will continue to run the states (including the monitor).
        """

        transition_message = ForcedTransitionMessage.from_json(message.data)
        if transition_message.kill_state_machine:
            self.is_running = False
            return False

        if self.transition_state:
            # todo(ragavsachdeva): Maybe the following function should run
            # in a seperate thread.
            self.transition_state.force_goto(transition_message)
        return True

    def service_preempt(self):
        self.is_running = False
        self._preempt_requested = False

    def connect_transition_state(self, transition_state):
        self.transition_state = transition_state

    def execute(self, userdata):
        self.is_running = True
        return super().execute(userdata)
