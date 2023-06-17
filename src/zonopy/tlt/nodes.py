import numpy as np

from zonopy.sets.hybrid_zonotopes import HybridZonotope

ALWAYS_SYMBOL = "\u25A1"
EVENTUALLY_SYMBOL = "\u25CA"
OR_SYMBOL = "\u2228"
AND_SYMBOL = "\u2227"
NEXT_SYMBOL = "\u25EF"
UNTIL_SYMBOL = "\u0055"
WUNTIL_SYMBOL = "\u0057"
REACH_SYMBOL = "\u211B"


class set_node:
    def __init__(self, set: HybridZonotope, set_name, is_root=True, is_leaf=True) -> None:
        self.set = set
        self.set_name = set_name
        self.is_root = is_root
        self.is_leaf = is_leaf
        self.parent = []
        self.children = []
        self.extra_name = ""

    def __str__(self) -> str:
        return f"{self.set_name}"

    def __repr__(self) -> str:
        return f"{self.set_name}"

    @property
    def name(self) -> str:
        return f"{self.set_name}{self.extra_name}"

    @property
    def label(self) -> str:
        return f"{self.set_name}"


class OR:
    def __init__(self, parent, *children) -> None:
        """
        *children: Tuple of set nodes
        """
        assert isinstance(parent, set_node), "OR parent must be a set node"
        assert len(children) > 1, "OR must have at least 2 children"
        for child in children:
            assert isinstance(child, set_node), "OR children must be set nodes"

        self.extra_name = ""

        self.parent = parent
        self.children = children

        self.parent.is_leaf = False

        self.parent.children.append(self)
        for child in self.children:
            child.parent = self
            child.is_root = False

    def __str__(self) -> str:
        children_name = f"{OR_SYMBOL}".join([c.__str__() for c in self.children])
        return f"{self.parent}_{children_name}{self.extra_name}"

    def __repr__(self) -> str:
        children_name = f"{OR_SYMBOL}".join([c.__str__() for c in self.children])
        return f"{self.parent}_{children_name}{self.extra_name}"

    @property
    def name(self) -> str:
        children_name = f"{OR_SYMBOL}".join([c.name for c in self.children])
        return f"{self.parent.name}_{children_name}{self.extra_name}"

    @property
    def label(self) -> str:
        return f"{OR_SYMBOL}"


class AND:
    def __init__(self, parent, *children) -> None:
        """
        *children: Tuple of set nodes
        """
        assert isinstance(parent, set_node), "AND parent must be a set node"
        assert len(children) > 1, "AND must have at least 2 children"
        for child in children:
            assert isinstance(child, set_node), "AND children must be set nodes"

        self.extra_name = ""

        self.parent = parent
        self.children = children

        self.parent.is_leaf = False

        self.parent.children.append(self)
        for child in self.children:
            child.parent = self
            child.is_root = False

    def __str__(self) -> str:
        children_name = f"{AND_SYMBOL}".join([c.name for c in self.children])
        return f"{self.parent.name}_{children_name}{self.extra_name}"

    def __repr__(self) -> str:
        children_name = f"{AND_SYMBOL}".join([c.name for c in self.children])
        return f"{self.parent.name}_{children_name}{self.extra_name}"

    @property
    def name(self) -> str:
        children_name = f"{AND_SYMBOL}".join([c.name for c in self.children])
        return f"{self.parent.name}_{children_name}{self.extra_name}"

    @property
    def label(self) -> str:
        return f"{AND_SYMBOL}"


class UNTIL:
    def __init__(self, parent, *children) -> None:
        assert isinstance(parent, set_node), "UNTIL parent must be a set node"
        assert len(children) == 1, "UNTIL must have exactly 1 children"
        for child in children:
            assert isinstance(child, set_node), "UNTIL children must be a set node"

        self.extra_name = ""

        self.parent = parent
        self.children = children

        self.parent.is_leaf = False

        self.parent.children.append(self)
        for child in self.children:
            child.parent = self
            child.is_root = False

    def __str__(self) -> str:
        children_name = f"{UNTIL_SYMBOL}".join([c.name for c in self.children])
        return f"{self.parent.name}_{children_name}{self.extra_name}"

    def __repr__(self) -> str:
        children_name = f"{UNTIL_SYMBOL}".join([c.name for c in self.children])
        return f"{self.parent.name}_{children_name}{self.extra_name}"

    @property
    def name(self) -> str:
        children_name = f"{UNTIL_SYMBOL}".join([c.name for c in self.children])
        return f"{self.parent.name}_{children_name}{self.extra_name}"

    @property
    def label(self) -> str:
        return f"{UNTIL_SYMBOL}"


class WUNTIL:
    # TODO: This is not supported yet
    def __init__(self, parent, *children) -> None:
        assert isinstance(parent, set_node), "WUNTIL parent must be a set node"
        assert len(children) == 1, "WUNTIL must have exactly 1 children"
        for child in children:
            assert isinstance(child, set_node), "WUNTIL children must be a set node"

        self.parent = parent
        self.children = children

    def __str__(self) -> str:
        return f"{WUNTIL_SYMBOL}"


class NEXT:
    # TODO: This is not supported yet
    def __init__(self, parent, *children) -> None:
        assert isinstance(parent, set_node), "NEXT parent must be a set node"
        assert len(children) == 1, "NEXT must have exactly 1 children"
        for child in children:
            assert isinstance(child, set_node), "NEXT children must be a set node"

        self.parent = parent
        self.children = children

    def __str__(self) -> str:
        return f"{NEXT_SYMBOL}"
