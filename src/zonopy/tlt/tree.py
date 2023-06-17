from ..tlt.nodes import OR, AND, UNTIL, set_node, WUNTIL, NEXT


class Tree(object):
    def __init__(self, *nodes) -> None:

        for n in nodes:
            assert isinstance(n, set_node) or isinstance(n, OR) or isinstance(n, AND) or isinstance(n, UNTIL) or isinstance(n, WUNTIL) or isinstance(n, NEXT), 'Nodes must be set nodes, OR, AND, UNTIL, WUNTIL, or NEXT'

        self.nodes = nodes

    @ property
    def root(self):
        for n in self.nodes:
            if isinstance(n, set_node):
                if n.is_root:
                    return n

    @ property
    def leaves(self):
        leaves = []
        for n in self.nodes:
            if isinstance(n, set_node):
                if n.is_leaf:
                    leaves.append(n)
        return leaves

    def get_nodes(self):
        nodes = list(self.nodes)
        return nodes

    def replace_node(self, old_node, new_node):
        # Detect parent and children of old_node
        parent = old_node.parent
        children = old_node.children

        # Replace old_node with new_node (First check if it has a parent)
        if parent:
            parent.children.remove(old_node)
            parent.children.append(new_node)

        for child in children:
            child.parent = new_node

        new_node.parent = parent
        new_node.children = children

        # Remove old_node from tree (Keep in mind it it a tuple)
        self.nodes = [n for n in self.nodes if n != old_node]

        # Add new_node to tree
        self.nodes.append(new_node)

        # Convert to tuple
        self.nodes = tuple(self.nodes)

