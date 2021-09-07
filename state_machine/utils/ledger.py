import threading
from itertools import chain

import rospy
from constants.services import SEND_BRAIN_A_MESSAGE_SERVICE_RETRIES
from easydict import EasyDict
from services.brain.communication_messages import CurrentStateMessage
from services.logger import GlobalLogger
from services.services import service_call

from srcp2_solution.srv import BrainMessage

lock = threading.Lock()


class Ledger:
    def __init__(
        self,
    ):
        self._all_states = []
        self._active_states = []
        self._next_subscriber_id = 0
        self._state_subscribers = EasyDict()
        self._logger = GlobalLogger(service_name=__name__, rover_name=None)
        rospy.wait_for_service("brain_service")
        self.send_brain_a_message = rospy.ServiceProxy("brain_service", BrainMessage)

    def subscribe_to_state(self, tree, event_type):
        with lock:
            state_subscription_key = tree + " " + event_type
            if state_subscription_key in self._state_subscribers.keys():
                self._state_subscribers[state_subscription_key].append(
                    self._next_subscriber_id
                )
            else:
                self._state_subscribers[state_subscription_key] = [
                    self._next_subscriber_id
                ]
            self._next_subscriber_id += 1
            return self._next_subscriber_id - 1

    def atomically_check_subscriptions_and_trigger_event(
        self, tree, subscriber_ids, event_type
    ):
        with lock:
            subscribers_status = [
                self._is_valid_subscriber(subscriber_id)
                for subscriber_id in subscriber_ids
            ]
            all_subscribers_released = not any(subscribers_status)
            if not all_subscribers_released:
                return False
            if event_type == "ENTER":
                self._entering_state(tree)
            elif event_type == "EXIT":
                self._exiting_state(tree)
            else:
                raise NotImplementedError()
            return True

    def force_exit_state(self, tree):
        if tree in self._active_states:
            self._exiting_state(tree)

    def _entering_state(self, tree):
        self._logger.info(f"Entering state: {tree}")
        self._active_states.append(tree)
        self._add_entry(tree, "ENTER")

    def _exiting_state(self, tree):
        self._logger.info(f"Exiting state: {tree}")
        if tree in self._active_states:
            self._active_states.remove(tree)
        else:
            self._logger.warning("Exiting state without an entry record!!")
        self._add_entry(tree, "EXIT")


    def _is_valid_subscriber(self, subscriber_id):
        self._release_active_and_inactive_subscribers()
        all_subscribers = self._get_all_subscribers()
        return subscriber_id in all_subscribers

    def _add_entry(self, tree, event_type):
        self._all_states.append(tree + " " + event_type)
        self._release_subscribers(tree, event_type)

    def _get_all_subscribers(self):
        return list(chain.from_iterable(self._state_subscribers.values()))

    def _get_all_subscribed_states(self):
        return [state_key.split()[0] for state_key in self._state_subscribers.keys()]

    def _release_active_and_inactive_subscribers(self):
        for active_state in self._active_states:
            self._release_subscribers(active_state, "ACTIVE")
        all_subscribed_states = self._get_all_subscribed_states()
        for subscribed_state in all_subscribed_states:
            if subscribed_state not in self._active_states:
                self._release_subscribers(subscribed_state, "INACTIVE")

    def _release_subscribers(self, tree, event_type):
        state_subscription_key = tree + " " + event_type
        if state_subscription_key not in self._state_subscribers.keys():
            return
        self._state_subscribers[state_subscription_key] = []

    def print(
        self,
    ):
        for key, value in self._all_states.items():
            self._logger.info(f"{key} {value}")
