def print_stacks(stacks):
    for i, stack in enumerate(stacks):
        print(f"Block(s) on stack: {stack}")
    print()

def move_block(stacks, from_stack, to_stack):
    block = stacks[from_stack].pop()
    stacks[to_stack].append(block)

initial_stacks = [[0], [1], [2]]
print("Initial state:")
print_stacks(initial_stacks)
goal_stacks = [[0, 1], [2]]
print("Goal state set.")
print_stacks(goal_stacks)


print("Performing Moves:")
move_block(initial_stacks, 0, 1)
print_stacks(initial_stacks)
move_block(initial_stacks, 2, 0)
print_stacks(initial_stacks)

move_block(initial_stacks, 1, 2)
print_stacks(initial_stacks)


move_block(initial_stacks, 1, 0)
print_stacks(initial_stacks)


move_block(initial_stacks, 2, 1)
print_stacks(initial_stacks) 9th