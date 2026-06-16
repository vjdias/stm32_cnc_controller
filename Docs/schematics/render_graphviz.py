from pathlib import Path
import argparse

from graphviz import Digraph

import importlib.util

spec = importlib.util.spec_from_file_location("cnc_gen", Path(__file__).with_name("generate_cnc_schematic.py"))
cnc_gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cnc_gen)

nodes = {inst["name"]: inst for inst in cnc_gen.INSTANCES}

parser = argparse.ArgumentParser(description="Render a Graphviz overview of the CNC interconnects")
parser.add_argument(
    "-f",
    "--format",
    action="append",
    dest="formats",
    help="Output format accepted by Graphviz (svg, png, pdf, ...). Can be passed multiple times.",
)
args = parser.parse_args()

formats = args.formats or ["svg"]

G = Digraph("CNC_Interconnects")
G.attr(rankdir="LR", fontsize="12")
G.attr("node", shape="box", style="rounded,filled", fillcolor="#f3f7ff", fontname="Liberation Sans")
G.attr("edge", arrowhead="none", fontname="Liberation Sans", fontsize="10")

for inst in cnc_gen.INSTANCES:
    label = f"{inst['ref']}\\n{cnc_gen.SYMBOLS[inst['name']].value}"
    G.node(inst["name"], label=label)

for net_name, endpoints in cnc_gen.NETS.items():
    hub = f"NET_{net_name}"
    G.node(hub, label=net_name, shape="ellipse", style="filled", fillcolor="#e0ecff")
    for inst_name, pin_name in endpoints:
        G.edge(inst_name, hub, label=pin_name)

output_path = Path(__file__).with_name("cnc_connections")

for fmt in formats:
    rendered = G.render(str(output_path), format=fmt, cleanup=True)
    print(f"Rendered graph to {rendered}")
