import re
from pathlib import Path


def parse_modes(file_path: Path):
    text = file_path.read_text()

    enum_match = re.search(r"enum\s+TestMode\s*{([^}]*)}", text, re.DOTALL)
    assert enum_match, "TestMode enum not found"
    enum_block = enum_match.group(1)
    lines = [l.strip() for l in enum_block.splitlines() if l.strip()]

    value = 0
    mode_count = None
    for line in lines:
        line = line.split("//")[0].strip().rstrip(',')
        if not line:
            continue
        if '=' in line:
            name, val_expr = line.split('=', 1)
            name = name.strip()
            val = int(val_expr.strip(), 0)
        else:
            name = line
            val = value
        if name == 'MODE_COUNT':
            mode_count = val
            break
        value = val + 1

    if mode_count is None:
        raise AssertionError('MODE_COUNT not found in enum')

    names_block = re.search(r"const\s+char\s*\*\s*modeNames\[\]\s*=\s*{([^}]*)}", text, re.DOTALL)
    assert names_block, "modeNames array not found"
    mode_names = re.findall(r'"(.*?)"', names_block.group(1))

    speeds_block = re.search(r"const\s+uint32_t\s+modeSpeeds\[\]\s*=\s*{([^}]*)}", text, re.DOTALL)
    assert speeds_block, "modeSpeeds array not found"
    speed_lines = [l.split('//')[0] for l in speeds_block.group(1).splitlines()]
    speed_lines = [l.strip().rstrip(',') for l in speed_lines if l.strip()]
    mode_speeds = [int(v, 0) for v in speed_lines]

    return mode_count, mode_names, mode_speeds


def test_mode_arrays_match_enum():
    file_path = Path(__file__).resolve().parents[1] / 'Stepper Control Example.ino'
    mode_count, mode_names, mode_speeds = parse_modes(file_path)

    assert len(mode_names) == mode_count
    assert len(mode_speeds) == mode_count
    assert len(mode_names) == len(mode_speeds)
