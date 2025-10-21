import math
import uuid
from pathlib import Path

OUTPUT_PATH = Path(__file__).with_name("cnc_connections.kicad_sch")

# Helper to format float to KiCad style

def fmt(value: float) -> str:
    if abs(value) < 1e-6:
        value = 0.0
    if float(value).is_integer():
        return f"{int(value)}"
    return f"{value:.3f}".rstrip('0').rstrip('.')


def make_uuid() -> str:
    return str(uuid.uuid4())


def format_uuid(u: str) -> str:
    return u


class PinDef:
    def __init__(self, number, name, x, y, orientation, length=5.08, pin_type="passive"):
        self.number = str(number)
        self.name = name
        self.x = x
        self.y = y
        self.orientation = orientation
        self.length = length
        self.pin_type = pin_type


class SymbolDef:
    def __init__(self, lib_id, ref_prefix, value, description, rect_start, rect_end, pins):
        self.lib_id = lib_id
        self.ref_prefix = ref_prefix
        self.value = value
        self.description = description
        self.rect_start = rect_start
        self.rect_end = rect_end
        self.pins = pins


SYMBOLS = {
    "RPI": SymbolDef(
        "CNC:RaspberryPi3APlus",
        "J",
        "Raspberry Pi 3 A+",
        "Key Raspberry Pi 3 A+ control header pins",
        (-12.0, 18.0),
        (12.0, -18.0),
        [
            PinDef(1, "3V3", -15.0, 15.0, 0),
            PinDef(2, "GND", -15.0, 10.0, 0),
            PinDef(3, "SPI_MOSI", 15.0, 15.0, 180),
            PinDef(4, "SPI_MISO", 15.0, 10.0, 180),
            PinDef(5, "SPI_SCK", 15.0, 5.0, 180),
            PinDef(6, "CS_STM32", 15.0, 0.0, 180),
            PinDef(7, "CS_TMC5160", 15.0, -5.0, 180),
            PinDef(8, "5V_USB", -15.0, 5.0, 0),
            PinDef(9, "RUN", -15.0, 0.0, 0),
        ],
    ),
    "STM32": SymbolDef(
        "CNC:STM32_B-L475E-IOT01A",
        "U",
        "B-L475E-IOT01A",
        "STM32L475 Discovery board application connector",
        (-20.0, 30.0),
        (20.0, -30.0),
        [
            PinDef(1, "3V3", -23.0, 25.0, 0),
            PinDef(2, "GND", -23.0, 20.0, 0),
            PinDef(3, "SPI_MOSI", -23.0, 15.0, 0),
            PinDef(4, "SPI_MISO", -23.0, 10.0, 0),
            PinDef(5, "SPI_SCK", -23.0, 5.0, 0),
            PinDef(6, "SPI_NSS", -23.0, 0.0, 0),
            PinDef(7, "STEP_X", 23.0, 20.0, 180),
            PinDef(8, "DIR_X", 23.0, 15.0, 180),
            PinDef(9, "EN_X", 23.0, 10.0, 180),
            PinDef(10, "STEP_Y", 23.0, 5.0, 180),
            PinDef(11, "DIR_Y", 23.0, 0.0, 180),
            PinDef(12, "EN_Y", 23.0, -5.0, 180),
            PinDef(13, "STEP_Z", 23.0, -10.0, 180),
            PinDef(14, "DIR_Z", 23.0, -15.0, 180),
            PinDef(15, "EN_Z", 23.0, -20.0, 180),
            PinDef(16, "ENC_XA", -23.0, -5.0, 0),
            PinDef(17, "ENC_XB", -23.0, -10.0, 0),
            PinDef(18, "ENC_YA", -23.0, -15.0, 0),
            PinDef(19, "ENC_YB", -23.0, -20.0, 0),
            PinDef(20, "ENC_ZA", -23.0, -25.0, 0),
            PinDef(21, "ENC_ZB", -23.0, -30.0, 0),
            PinDef(22, "ESTOP", 23.0, 25.0, 180),
            PinDef(23, "LIM_X", 23.0, 30.0, 180),
            PinDef(24, "LIM_Y", 23.0, 35.0, 180),
            PinDef(25, "LIM_Z", 23.0, 40.0, 180),
        ],
    ),
    "TMC": SymbolDef(
        "CNC:TMC5160_DriverBank",
        "A",
        "TMC5160 Drivers",
        "Aggregate connections to three TMC5160 driver modules",
        (-18.0, 30.0),
        (18.0, -30.0),
        [
            PinDef(1, "3V3", -21.0, 25.0, 0),
            PinDef(2, "GND", -21.0, 20.0, 0),
            PinDef(3, "SPI_MOSI", -21.0, 15.0, 0),
            PinDef(4, "SPI_MISO", -21.0, 10.0, 0),
            PinDef(5, "SPI_SCK", -21.0, 5.0, 0),
            PinDef(6, "CS_TMC5160", -21.0, 0.0, 0),
            PinDef(7, "STEP_X", 21.0, 20.0, 180),
            PinDef(8, "DIR_X", 21.0, 15.0, 180),
            PinDef(9, "EN_X", 21.0, 10.0, 180),
            PinDef(10, "STEP_Y", 21.0, 5.0, 180),
            PinDef(11, "DIR_Y", 21.0, 0.0, 180),
            PinDef(12, "EN_Y", 21.0, -5.0, 180),
            PinDef(13, "STEP_Z", 21.0, -10.0, 180),
            PinDef(14, "DIR_Z", 21.0, -15.0, 180),
            PinDef(15, "EN_Z", 21.0, -20.0, 180),
        ],
    ),
    "ENC": SymbolDef(
        "CNC:EncoderBundle",
        "P",
        "Incremental Encoders",
        "Differential encoder channels for X/Y/Z axes",
        (-15.0, 15.0),
        (15.0, -15.0),
        [
            PinDef(1, "ENC_XA", 18.0, 10.0, 180),
            PinDef(2, "ENC_XB", 18.0, 5.0, 180),
            PinDef(3, "ENC_YA", 18.0, 0.0, 180),
            PinDef(4, "ENC_YB", 18.0, -5.0, 180),
            PinDef(5, "ENC_ZA", 18.0, -10.0, 180),
            PinDef(6, "ENC_ZB", 18.0, -15.0, 180),
            PinDef(7, "5V", -18.0, 10.0, 0),
            PinDef(8, "GND", -18.0, 5.0, 0),
        ],
    ),
    "SAFETY": SymbolDef(
        "CNC:SafetyInterlocks",
        "SW",
        "Safety Inputs",
        "Emergency stop and limit switches",
        (-12.0, 12.0),
        (12.0, -12.0),
        [
            PinDef(1, "ESTOP", -15.0, 10.0, 0),
            PinDef(2, "LIM_X", -15.0, 5.0, 0),
            PinDef(3, "LIM_Y", -15.0, 0.0, 0),
            PinDef(4, "LIM_Z", -15.0, -5.0, 0),
            PinDef(5, "GND", 15.0, 10.0, 180),
            PinDef(6, "GND", 15.0, 5.0, 180),
            PinDef(7, "3V3", 15.0, 0.0, 180),
        ],
    ),
}

# Instances placed on the sheet
INSTANCES = [
    {"name": "RPI", "ref": "J1", "at": (-120.0, 60.0)},
    {"name": "STM32", "ref": "U1", "at": (0.0, 60.0)},
    {"name": "TMC", "ref": "A1", "at": (120.0, 60.0)},
    {"name": "ENC", "ref": "P1", "at": (0.0, -40.0)},
    {"name": "SAFETY", "ref": "SW1", "at": (120.0, -40.0)},
]

PROJECT_NAME = "CNC_Interconnects"

# Net assignments per instance pin
NETS = {
    "3V3": [
        ("RPI", "3V3"),
        ("STM32", "3V3"),
        ("TMC", "3V3"),
        ("SAFETY", "3V3"),
    ],
    "GND": [
        ("RPI", "GND"),
        ("STM32", "GND"),
        ("TMC", "GND"),
        ("ENC", "GND"),
        ("SAFETY", "GND"),
    ],
    "SPI_MOSI": [
        ("RPI", "SPI_MOSI"),
        ("STM32", "SPI_MOSI"),
        ("TMC", "SPI_MOSI"),
    ],
    "SPI_MISO": [
        ("RPI", "SPI_MISO"),
        ("STM32", "SPI_MISO"),
        ("TMC", "SPI_MISO"),
    ],
    "SPI_SCK": [
        ("RPI", "SPI_SCK"),
        ("STM32", "SPI_SCK"),
        ("TMC", "SPI_SCK"),
    ],
    "CS_STM32": [
        ("RPI", "CS_STM32"),
        ("STM32", "SPI_NSS"),
    ],
    "CS_TMC5160": [
        ("RPI", "CS_TMC5160"),
        ("TMC", "CS_TMC5160"),
    ],
    "STEP_X": [
        ("STM32", "STEP_X"),
        ("TMC", "STEP_X"),
    ],
    "DIR_X": [
        ("STM32", "DIR_X"),
        ("TMC", "DIR_X"),
    ],
    "EN_X": [
        ("STM32", "EN_X"),
        ("TMC", "EN_X"),
    ],
    "STEP_Y": [
        ("STM32", "STEP_Y"),
        ("TMC", "STEP_Y"),
    ],
    "DIR_Y": [
        ("STM32", "DIR_Y"),
        ("TMC", "DIR_Y"),
    ],
    "EN_Y": [
        ("STM32", "EN_Y"),
        ("TMC", "EN_Y"),
    ],
    "STEP_Z": [
        ("STM32", "STEP_Z"),
        ("TMC", "STEP_Z"),
    ],
    "DIR_Z": [
        ("STM32", "DIR_Z"),
        ("TMC", "DIR_Z"),
    ],
    "EN_Z": [
        ("STM32", "EN_Z"),
        ("TMC", "EN_Z"),
    ],
    "ENC_XA": [
        ("STM32", "ENC_XA"),
        ("ENC", "ENC_XA"),
    ],
    "ENC_XB": [
        ("STM32", "ENC_XB"),
        ("ENC", "ENC_XB"),
    ],
    "ENC_YA": [
        ("STM32", "ENC_YA"),
        ("ENC", "ENC_YA"),
    ],
    "ENC_YB": [
        ("STM32", "ENC_YB"),
        ("ENC", "ENC_YB"),
    ],
    "ENC_ZA": [
        ("STM32", "ENC_ZA"),
        ("ENC", "ENC_ZA"),
    ],
    "ENC_ZB": [
        ("STM32", "ENC_ZB"),
        ("ENC", "ENC_ZB"),
    ],
    "ESTOP": [
        ("STM32", "ESTOP"),
        ("SAFETY", "ESTOP"),
    ],
    "LIM_X": [
        ("STM32", "LIM_X"),
        ("SAFETY", "LIM_X"),
    ],
    "LIM_Y": [
        ("STM32", "LIM_Y"),
        ("SAFETY", "LIM_Y"),
    ],
    "LIM_Z": [
        ("STM32", "LIM_Z"),
        ("SAFETY", "LIM_Z"),
    ],
    "5V_AUX": [
        ("RPI", "5V_USB"),
        ("ENC", "5V"),
    ],
    "RUN": [
        ("RPI", "RUN"),
    ],
}


# Build lookup for pins with metadata
pin_lookup = {}
for inst in INSTANCES:
    sym = SYMBOLS[inst["name"]]
    pin_map = {}
    for pin in sym.pins:
        pin_map[pin.name] = {
            "def": pin,
            "uuid": make_uuid(),
        }
    inst["uuid"] = make_uuid()
    inst["pin_map"] = pin_map

# Compose lib symbols definitions
lib_entries = []
symbol_instance_refs = {key: [] for key in SYMBOLS.keys()}
for inst in INSTANCES:
    symbol_instance_refs[inst["name"]].append(inst["ref"])

sheet_uuid = make_uuid()

for key, sym in SYMBOLS.items():
    rect_start = sym.rect_start
    rect_end = sym.rect_end
    symbol_uuid = make_uuid()
    body = []
    body.append(f"                (symbol \"{sym.lib_id}\"")
    body.append(f"                        (uuid {format_uuid(symbol_uuid)})")
    body.append("                        (pin_names"
                "\n                                (offset 1.27)"
                "\n                        )")
    body.append("                        (exclude_from_sim no)")
    body.append("                        (in_bom yes)")
    body.append("                        (on_board yes)")
    body.append(f"                        (property \"Reference\" \"{sym.ref_prefix}\""
                f"\n                                (at 0 {fmt(rect_end[1]-2)} 0)"
                "\n                                (effects"
                "\n                                        (font"
                "\n                                                (size 1.27 1.27)"
                "\n                                        )"
                "\n                                        (justify left)"
                "\n                                )"
                "\n                        )")
    body.append(f"                        (property \"Value\" \"{sym.value}\""
                f"\n                                (at 0 {fmt(rect_start[1]+2)} 0)"
                "\n                                (effects"
                "\n                                        (font"
                "\n                                                (size 1.27 1.27)"
                "\n                                        )"
                "\n                                        (justify left)"
                "\n                                )"
                "\n                        )")
    body.append("                        (property \"Footprint\" \"\""
                "\n                                (at 0 0 0)"
                "\n                                (effects"
                "\n                                        (font"
                "\n                                                (size 1.27 1.27)"
                "\n                                        )"
                "\n                                        (hide yes)"
                "\n                                )"
                "\n                        )")
    body.append("                        (property \"Datasheet\" \"\""
                "\n                                (at 0 0 0)"
                "\n                                (effects"
                "\n                                        (font"
                "\n                                                (size 1.27 1.27)"
                "\n                                        )"
                "\n                                        (hide yes)"
                "\n                                )"
                "\n                        )")
    body.append("                        (property \"ki_keywords\" \"\""
                "\n                                (at 0 0 0)"
                "\n                                (effects"
                "\n                                        (font"
                "\n                                                (size 1.27 1.27)"
                "\n                                        )"
                "\n                                        (hide yes)"
                "\n                                )"
                "\n                        )")
    body.append("                        (property \"ki_description\" \"\""
                "\n                                (at 0 0 0)"
                "\n                                (effects"
                "\n                                        (font"
                "\n                                                (size 1.27 1.27)"
                "\n                                        )"
                "\n                                        (hide yes)"
                "\n                                )"
                "\n                        )")
    body.append("                        (property \"ki_fp_filters\" \"\""
                "\n                                (at 0 0 0)"
                "\n                                (effects"
                "\n                                        (font"
                "\n                                                (size 1.27 1.27)"
                "\n                                        )"
                "\n                                        (hide yes)"
                "\n                                )"
                "\n                        )")
    body.append(f"                        (property \"Description\" \"{sym.description}\""
                "\n                                (at 0 0 0)"
                "\n                                (effects"
                "\n                                        (font"
                "\n                                                (size 1.27 1.27)"
                "\n                                        )"
                "\n                                        (hide yes)"
                "\n                                )"
                "\n                        )")
    body.append(f"                        (symbol \"{sym.lib_id}_body\""
                f"\n                                (rectangle"
                f"\n                                        (start {fmt(rect_start[0])} {fmt(rect_start[1])})"
                f"\n                                        (end {fmt(rect_end[0])} {fmt(rect_end[1])})"
                "\n                                        (stroke"
                "\n                                                (width 0.254)"
                "\n                                                (type default)"
                "\n                                        )"
                "\n                                        (fill"
                "\n                                                (type none)"
                "\n                                        )"
                "\n                                )"
                "\n                        )")
    # pins
    pin_lines = [f"                        (symbol \"{sym.lib_id}_pins\""]
    for pin in sym.pins:
        pin_uuid = make_uuid()
        pin_lines.append(
            "                                (pin {pin_type} line".format(pin_type=pin.pin_type)
        )
        pin_lines.append(
            f"                                        (uuid {format_uuid(pin_uuid)})"
        )
        pin_lines.append(
            f"                                        (at {fmt(pin.x)} {fmt(pin.y)} {fmt(pin.orientation)})"
        )
        pin_lines.append(
            f"                                        (length {fmt(pin.length)})"
        )
        pin_lines.append(
            f"                                        (name \"{pin.name}\""
            "\n                                                (effects"
            "\n                                                        (font"
            "\n                                                                (size 1.27 1.27)"
            "\n                                                        )"
            "\n                                                        (justify left)"
            "\n                                                )"
            "\n                                        )"
        )
        pin_lines.append(
            f"                                        (number \"{pin.number}\""
            "\n                                                (effects"
            "\n                                                        (font"
            "\n                                                                (size 1.27 1.27)"
            "\n                                                        )"
            "\n                                                        (hide yes)"
            "\n                                                )"
            "\n                                        )"
        )
        pin_lines.append("                                )")
    pin_lines.append("                        )")
    body.extend(pin_lines)
    instances_lines = ["                        (instances",
                      f"                                (project \"{PROJECT_NAME}\""]
    for ref in symbol_instance_refs[key]:
        instances_lines.append(
            f"                                        (path \"/{sheet_uuid}\" (reference \"{ref}\") (unit 1))"
        )
    instances_lines.append("                                )")
    instances_lines.append("                        )")
    body.extend(instances_lines)
    body.append("                        (embedded_fonts no)")
    body.append("                )")
    lib_entries.append("\n".join(body))

# Compose schematic symbols instances
symbol_instances = []
for inst in INSTANCES:
    sym = SYMBOLS[inst["name"]]
    inst_lines = ["        (symbol",
                  f"                (lib_id \"{sym.lib_id}\")",
                  f"                (at {fmt(inst['at'][0])} {fmt(inst['at'][1])} 0)",
                  "                (unit 1)",
                  "                (exclude_from_sim no)",
                  "                (in_bom yes)",
                  "                (on_board yes)",
                  "                (dnp no)",
                  "                (fields_autoplaced yes)",
                  f"                (uuid {format_uuid(inst['uuid'])})",
                  f"                (property \"Reference\" \"{inst['ref']}\"",
                  f"                        (at {fmt(inst['at'][0])} {fmt(inst['at'][1] + sym.rect_start[1] + 5)} 0)",
                  "                        (effects",
                  "                                (font",
                  "                                        (size 1.27 1.27)",
                  "                                )",
                  "                        )",
                  "                )",
                  f"                (property \"Value\" \"{sym.value}\"",
                  f"                        (at {fmt(inst['at'][0])} {fmt(inst['at'][1] + sym.rect_end[1] - 5)} 0)",
                  "                        (effects",
                  "                                (font",
                  "                                        (size 1.27 1.27)",
                  "                                )",
                  "                        )",
                  "                )",
                  "                (property \"Footprint\" \"\"",
                  f"                        (at {fmt(inst['at'][0])} {fmt(inst['at'][1])} 0)",
                  "                        (effects",
                  "                                (font",
                  "                                        (size 1.27 1.27)",
                  "                                )",
                  "                                (hide yes)",
                  "                        )",
                  "                )",
                  "                (property \"Datasheet\" \"\"",
                  f"                        (at {fmt(inst['at'][0])} {fmt(inst['at'][1])} 0)",
                  "                        (effects",
                  "                                (font",
                  "                                        (size 1.27 1.27)",
                  "                                )",
                  "                                (hide yes)",
                  "                        )",
                  "                )",
                  f"                (property \"Description\" \"{sym.description}\"",
                  f"                        (at {fmt(inst['at'][0])} {fmt(inst['at'][1])} 0)",
                  "                        (effects",
                  "                                (font",
                  "                                        (size 1.27 1.27)",
                  "                                )",
                  "                                (hide yes)",
                  "                        )",
                  "                )",
                 ]
    for pin_name, pin_info in inst["pin_map"].items():
        inst_lines.append(
            f"                (pin \"{pin_info['def'].number}\" (uuid {format_uuid(pin_info['uuid'])}))"
        )
    inst_lines.extend([
        "                (instances",
        f"                        (project \"{PROJECT_NAME}\"",
        f"                                (path \"/{sheet_uuid}\"",
        f"                                        (reference \"{inst['ref']}\")",
        "                                        (unit 1)",
        "                                )",
        "                        )",
        "                )",
        "        )",
    ])
    symbol_instances.append("\n".join(inst_lines))

# Function to compute absolute pin connection location
def pin_position(inst, pin_name):
    pin = inst["pin_map"][pin_name]["def"]
    angle = math.radians(pin.orientation)
    # Rotation is zero for all instances, so this is simple
    return (inst["at"][0] + pin.x, inst["at"][1] + pin.y, pin.orientation)

wires = []
global_labels = []
for net_name, endpoints in NETS.items():
    for inst_name, pin_name in endpoints:
        inst = next(i for i in INSTANCES if i["name"] == inst_name)
        x, y, orientation = pin_position(inst, pin_name)
        if orientation == 0:
            end = (x + 7.0, y)
            label_pos = (x + 9.0, y)
            angle = 0
            justify = "left"
        elif orientation == 180:
            end = (x - 7.0, y)
            label_pos = (x - 9.0, y)
            angle = 180
            justify = "right"
        else:
            end = (x, y - 7.0)
            label_pos = (x, y - 9.0)
            angle = 90
            justify = "center"
        wires.append(
            "        (wire\n"
            "                (pts\n"
            f"                        (xy {fmt(x)} {fmt(y)}) (xy {fmt(end[0])} {fmt(end[1])})\n"
            "                )\n"
            "                (stroke\n"
            "                        (width 0)\n"
            "                        (type solid)\n"
            "                )\n"
            f"                (uuid {format_uuid(make_uuid())})\n"
            "        )"
        )
        effects = "(font\n                                (size 1.27 1.27)\n                        )"
        justify_line = ""
        if justify == "right":
            justify_line = "\n                        (justify right)"
        elif justify == "center":
            justify_line = "\n                        (justify center)"
        global_labels.append(
            f"        (global_label \"{net_name}\"\n"
            f"                (shape input)\n"
            f"                (at {fmt(label_pos[0])} {fmt(label_pos[1])} {fmt(angle)})\n"
            "                (fields_autoplaced yes)\n"
            "                (effects\n"
            f"                        {effects}{justify_line}\n"
            "                )\n"
            f"                (uuid {format_uuid(make_uuid())})\n"
            "        )"
        )

content = [
    "(kicad_sch",
    "        (version 20221206)",
    "        (generator eeschema)",
    f"        (uuid {format_uuid(make_uuid())})",
    "        (paper \"A3\")",
    "        (title_block",
    "                (title \"CNC Controller Interconnects\")",
    "                (rev \"1.0\")",
    "                (company \"Auto-generated via kicad-skip script\")",
    "        )",
    "        (lib_symbols",
]
content.extend(lib_entries)
content.append("        )")
content.append("        (sheet")
content.append("                (number 1)")
content.append("                (name \"\")")
content.append(f"                (uuid {format_uuid(sheet_uuid)})")
content.append("                (property \"Sheetname\" \"\""
               "\n                        (at 0 0 0)"
               "\n                        (effects"
               "\n                                (font"
               "\n                                        (size 1.27 1.27)"
               "\n                                )"
               "\n                                (justify left)"
               "\n                        )"
               "\n                )")
content.append("                (property \"Sheetfile\" \"cnc_connections.kicad_sch\""
               "\n                        (at 0 -5 0)"
               "\n                        (effects"
               "\n                                (font"
               "\n                                        (size 1.27 1.27)"
               "\n                                )"
               "\n                                (justify left)"
               "\n                        )"
               "\n                )")
content.extend(symbol_instances)
content.extend(wires)
content.extend(global_labels)
content.append("        )")
content.append("        (sheet_instances")
content.append("                (path \"/\" (page \"1\"))")
content.append("        )")
content.append(")")

OUTPUT_PATH.write_text("\n".join(content) + "\n")
print(f"Wrote schematic to {OUTPUT_PATH}")
