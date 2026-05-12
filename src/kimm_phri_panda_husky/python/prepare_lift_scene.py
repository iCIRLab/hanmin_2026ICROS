#!/usr/bin/env python3
"""Create a temporary support-on/off cooperative scene for grasp/lift tests."""

import argparse
import os
import sys
import xml.etree.ElementTree as ET


SUPPORT_BODIES = {
    "object_start_support_front",
    "object_start_support_rear",
}


def remove_support_bodies(root):
    removed = []
    parent = {child: p for p in root.iter() for child in list(p)}
    for body in list(root.iter("body")):
        name = body.get("name")
        if name in SUPPORT_BODIES and body in parent:
            parent[body].remove(body)
            removed.append(name)
    return removed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        default="src/kimm_robots_description/husky_description/husky_coop/coop_transport_scene.xml",
    )
    parser.add_argument(
        "--output",
        default="src/kimm_robots_description/husky_description/husky_coop/coop_transport_scene_lift_generated.xml",
    )
    parser.add_argument("--support", choices=["on", "off"], default="off")
    args = parser.parse_args()

    tree = ET.parse(args.source)
    root = tree.getroot()
    removed = []
    if args.support == "off":
        removed = remove_support_bodies(root)

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=False)

    print("wrote", args.output)
    print("support", args.support)
    print("removed", ",".join(removed) if removed else "none")
    return 0


if __name__ == "__main__":
    sys.exit(main())
