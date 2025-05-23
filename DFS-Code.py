graph = {
    'A': ['B'],
    'B': ['D', 'E'],
    'D': [],
    'E': ['F'],
    'F': []
}

def dfs(graph, current_node, target_node, path):
    path.append(current_node)  
   
    if current_node == target_node:
        return True
   

    for neighbor in graph[current_node]:
        if dfs(graph, neighbor, target_node, path):
            return True
   

    path.pop()
    return False


start_node = 'A'
target_node = 'F'
path = []

if dfs(graph, start_node, target_node, path):
    print(f"Path from {start_node} to {target_node}: {'->'.join(path)}")
else:
    print(f"No path found from {start_node} to {target_node}")