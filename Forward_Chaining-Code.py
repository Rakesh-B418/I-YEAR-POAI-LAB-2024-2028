def forward_chaining(facts, rules, goal):
    inferred = set(facts)
    applied = True

    while applied:
        applied = False
        for rule in rules:
            head = rule[0]
            body = rule[1:]

            if head not in inferred and all(premise in inferred for premise in body):
                inferred.add(head)
                applied = True

    return goal in inferred
rules = [
    ['c', 'a', 'b'],
    ['d', 'b'],
    ['f', 'c', 'd']
]

# Initial facts
facts = ['a', 'b']
goals = ['f', 'e', 'd']

for goal in goals:
    if forward_chaining(facts.copy(), rules, goal):
        print(f"The goal '{goal}' can be achieved.")
    else:
        print(f"The goal '{goal}' cannot be achieved.") 8th