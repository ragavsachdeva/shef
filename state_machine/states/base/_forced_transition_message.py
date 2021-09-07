from dataclasses import dataclass
from typing import List, Literal

from dataclasses_json import dataclass_json


@dataclass_json
@dataclass
class ForcedTransitionMessage:
    access_control: Literal["preempt_if_in_given_states", "ignore_if_in_given_states"]
    potential_current_states: List[str]
    target_state: str
    kill_state_machine: bool = False
