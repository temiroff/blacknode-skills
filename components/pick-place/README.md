# Pick Place

Component of `blacknode-skills`.

Node sources for this component belong in this folder. Until they move here,
nodes claim the component inline:

    @node(name="MyNode", component="pick-place", ...)

Once sources live here, declare the folder in `blacknode-package.toml`:

    [components.pick-place]
    nodes = ["components/pick-place/nodes"]

and the inline `component=` argument can be dropped — the loader infers it
from the directory.
