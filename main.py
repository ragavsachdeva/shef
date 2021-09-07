import rospy
from state_machine.utils.ledger import Ledger
from state_machine.utils.recipe_parser import construct_state_machine, load_recipe


def state_machine_main(recipe_name: str):
    rospy.init_node(f"state_machine_{recipe_name.split('/')[-1]}")
    recipe = load_recipe(recipe_name)
    ledger = Ledger()
    challenge_state_machine = construct_state_machine(recipe, [recipe_name], ledger)
    challenge_state_machine.execute()

# state_machine_main(recipe_name="<name_of_your_recipe>")