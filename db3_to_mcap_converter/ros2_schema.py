"""Generate ROS2-compliant msg-def text for MCAP from a rosbags Typestore.

Foxglove / mcap_ros2 require:
- `builtin_interfaces/Time` instead of the ROS1 `time` alias
- One MSG: block per dependency, separated by 80 `=` chars
- `package/msg/Type` references shortened to `package/Type` in field types
"""

from typing import Dict, List, Set
from rosbags.interfaces import Nodetype


SEP = '=' * 80


def _short_typename(name: str, context_pkg: str = '') -> str:
    """Convert a rosbags type name to its canonical ROS2 schema-text form.

    'std_msgs/msg/Header' with context 'sensor_msgs' -> 'std_msgs/Header'
    'sensor_msgs/msg/NavSatStatus' with context 'sensor_msgs' -> 'NavSatStatus'
    Same-package references must be UNQUALIFIED to match ROS2's IDL convention,
    or stricter parsers (Foxglove Studio's @foxglove/rosmsg2) will not resolve
    them against the embedded MSG: blocks and field offsets get corrupted.
    """
    parts = name.split('/')
    if len(parts) == 3 and parts[1] == 'msg':
        pkg, _, msg = parts
    elif len(parts) == 2:
        pkg, msg = parts
    else:
        return name
    if context_pkg and pkg == context_pkg:
        return msg
    return f'{pkg}/{msg}'


def _node_to_str(node, context_pkg: str = '') -> str:
    """Render a field type node as the string used in a .msg file."""
    nt, payload = node
    if nt == Nodetype.BASE:
        base, length = payload
        return base
    if nt == Nodetype.NAME:
        return _short_typename(payload, context_pkg)
    if nt == Nodetype.ARRAY:
        inner, n = payload
        return f'{_node_to_str(inner, context_pkg)}[{n}]'
    if nt == Nodetype.SEQUENCE:
        inner, n = payload
        return f'{_node_to_str(inner, context_pkg)}[]' if not n else f'{_node_to_str(inner, context_pkg)}[<={n}]'
    raise ValueError(f'unknown node type: {nt}')


def _collect_dep_names(node, out: Set[str]) -> None:
    nt, payload = node
    if nt == Nodetype.NAME:
        out.add(payload)
    elif nt in (Nodetype.ARRAY, Nodetype.SEQUENCE):
        _collect_dep_names(payload[0], out)


def _render_msg_body(typestore, full_name: str) -> str:
    """Render one MSG: block body (constants then fields)."""
    consts, fields = typestore.fielddefs[full_name]
    # Determine context package for unqualified same-package references
    parts = full_name.split('/')
    context_pkg = parts[0] if len(parts) >= 2 else ''
    lines: List[str] = []
    for cname, ctype, cval in consts:
        if ctype == 'string':
            lines.append(f'{ctype} {cname} = "{cval}"')
        else:
            lines.append(f'{ctype} {cname} = {cval}')
    for fname, fnode in fields:
        lines.append(f'{_node_to_str(fnode, context_pkg)} {fname}')
    return '\n'.join(lines)


def generate_ros2_schema(typestore, root_typename: str) -> str:
    """Return the concatenated MCAP-ready msg-def text for `root_typename`.

    `root_typename` should be the rosbags-style 'package/msg/Type'.
    """
    # BFS over the dependency graph
    visited: Set[str] = set()
    order: List[str] = []
    queue = [root_typename]
    while queue:
        cur = queue.pop(0)
        if cur in visited:
            continue
        if cur not in typestore.fielddefs:
            # Skip primitives (shouldn't show up here) — they have no fielddef
            continue
        visited.add(cur)
        order.append(cur)
        consts, fields = typestore.fielddefs[cur]
        deps: Set[str] = set()
        for _, fnode in fields:
            _collect_dep_names(fnode, deps)
        for d in sorted(deps):
            if d not in visited:
                queue.append(d)

    # Root body first, then dependencies
    parts: List[str] = []
    parts.append(_render_msg_body(typestore, order[0]))
    for dep in order[1:]:
        parts.append(SEP)
        parts.append(f'MSG: {_short_typename(dep)}')
        parts.append(_render_msg_body(typestore, dep))
    return '\n'.join(parts) + '\n'


if __name__ == '__main__':
    # smoke test
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    for tn in ['sensor_msgs/msg/Imu', 'sensor_msgs/msg/NavSatFix', 'nav_msgs/msg/Odometry']:
        print(f'=== {tn} ===')
        print(generate_ros2_schema(ts, tn))
        print()
