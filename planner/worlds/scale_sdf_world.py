#!/usr/bin/env python3
import xml.etree.ElementTree as ET

SCALE = 0.7
INPUT_FILE = "larger_garage.world"
OUTPUT_FILE = "larger_garage_scaled.world"

tree = ET.parse(INPUT_FILE)
root = tree.getroot()

def parse_floats(text):
    return [float(x) for x in text.strip().split()]

def fmt(nums):
    out = []
    for x in nums:
        s = f"{x:.6f}".rstrip('0').rstrip('.')
        if s == "-0":
            s = "0"
        out.append(s)
    return " ".join(out)

def scale_pose_xy(elem, scale):
    if elem is None or elem.text is None:
        return
    vals = parse_floats(elem.text)
    if len(vals) != 6:
        return
    vals[0] *= scale
    vals[1] *= scale
    # z roll pitch yaw 不变
    elem.text = fmt(vals)

def scale_box_size_xy(elem, scale):
    if elem is None or elem.text is None:
        return
    vals = parse_floats(elem.text)
    if len(vals) != 3:
        return
    # x: 墙长, y: 墙厚, z: 墙高
    vals[0] *= scale
    vals[1] *= scale
    # 高度不变
    elem.text = fmt(vals)

world = root.find("world")
if world is None:
    raise RuntimeError("Cannot find <world>")

maze_model = None
for model in world.findall("model"):
    if model.get("name") == "map8":
        maze_model = model
        break

if maze_model is None:
    raise RuntimeError("Cannot find <model name='mazetest'>")

# 不改 mazetest 自身 pose，保持整个迷宫锚点不变
# 只改每个 wall link 的局部 pose
for link in maze_model.findall("link"):
    link_pose = link.find("pose")
    scale_pose_xy(link_pose, SCALE)

    # 同时改 collision / visual 下的 box size
    for box_size in link.findall(".//box/size"):
        scale_box_size_xy(box_size, SCALE)

# 删除 state，避免旧状态覆盖新定义
state_elem = world.find("state")
if state_elem is not None:
    world.remove(state_elem)

tree.write(OUTPUT_FILE, encoding="utf-8", xml_declaration=True)
print(f"Scaled file written to: {OUTPUT_FILE}")