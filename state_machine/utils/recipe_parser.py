import copy
import os
import pathlib
from typing import Tuple

import yaml
from easydict import EasyDict

from .. import states


def load_recipe(file_name: str) -> EasyDict:
    with open(get_file_path(file_name), "r") as stream:
        recipe_yml = yaml.safe_load(stream)
    recipe = EasyDict()
    for k, v in recipe_yml.items():
        recipe[k] = v
    return recipe


def get_file_path(file_name: str) -> str:
    return os.path.join(
        pathlib.Path(__file__).parent.absolute(),
        pathlib.Path(f"../../recipes/{file_name}.yml"),
    )


def construct_state_machine(recipe, seen_imports, ledger, tree=""):
    """
    Recursively construct a state machine from a recipe yaml file.

    Args:
        recipe(EasyDict): Recipe tree of the current state machine and it's children.
        seen_imports(List[str]): List of all imports previously seen in this branch.
        ledger(Ledger): Dictionary of past states used to synchronise robots.
        tree(str): Path to current state machine path.

    Returns:
        BaseStateMachine: The constructed state machine which can either be a
        Basestate, SimpleStateMachine, or ConcurrentStateMachine.
    """
    for state_name, state_info in recipe.items():
        tree = tree + "/" + state_name
        current_state_outcome = state_name + "_EXITED"

        new_seen_imports = copy.deepcopy(seen_imports)

        if "import" in state_info.keys():
            import_name = state_info["import"]
            if import_name in new_seen_imports:
                raise RuntimeError(
                    "Nested imports. "
                    f"{import_name} has already been imported in this branch."
                )

            new_seen_imports.append(import_name)
            child_recipe = load_recipe(import_name)
            _, state_info = list(child_recipe.items())[0]

        conditions = state_info.get("conditions", None)

        children = []
        if "child_states" in state_info.keys():
            for child_info in state_info["child_states"].items():
                child_state_machine = construct_state_machine(
                    to_easy_dict(child_info), new_seen_imports, ledger, tree
                )
                children.append(child_state_machine)

        if state_info["type"] == "SimpleStateMachine":
            transition_state_outcomes = []
            for child in children:
                transition_state_outcomes.append(child.name)

            transition_state_outcomes.append(current_state_outcome)
            transition_state = eval(f"states.{state_info['transition_state']}")(
                "transition",
                tree=tree,
                outcomes=transition_state_outcomes,
                conditions=conditions,
                ledger=ledger,
                **state_info.get("args", dict()),
            )
            state_machine = states.SimpleStateMachine(
                state_name, tree=tree, outcome=current_state_outcome
            )

            for child in children:
                if isinstance(child, states.BaseState):
                    child.connect_data_store(transition_state.data_store)
                    child.connect_ledger(transition_state.ledger)

            state_machine.add_states(children, transition_state)
            return state_machine

        if state_info["type"] == "ConcurrentStateMachine":
            state_machine = eval(f"states.{state_info['type']}")(
                state_name,
                tree=tree,
                outcome=current_state_outcome,
                **state_info.get("args", dict()),
            )
            state_machine.add_states(children)
            return state_machine

        state_machine = eval(f"states.{state_info['type']}")(
            state_name,
            tree=tree,
            outcome=current_state_outcome,
            conditions=conditions,
            **state_info.get("args", dict()),
        )
        state_machine.add_states(children)

        return state_machine


def to_easy_dict(single_tuple: Tuple[str, EasyDict]) -> EasyDict:
    easy_dict = EasyDict()
    k, v = single_tuple
    easy_dict[k] = v
    return easy_dict
