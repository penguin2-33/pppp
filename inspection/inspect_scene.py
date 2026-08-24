# -*- coding: utf-8 -*-
"""诊断工具：转储当前场景对象树到 UTF-8 文件。"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

OUT = "inspect_scene_output.txt"
client = RemoteAPIClient()
sim = client.require("sim")

def typestr(h):
    return {0: "shape", 1: "joint", 2: "graph", 3: "camera", 4: "dummy",
            5: "prox", 6: "path", 7: "vision", 8: "mill", 9: "force",
            10: "light", 11: "mirror", 12: "octree", 13: "pointcloud"}.get(sim.getObjectType(h), "?")

def bbox_str(h):
    try:
        bmin = sim.getObjectFloatParam(h, sim.shapefloatparam_init_bbox_min)
        bmax = sim.getObjectFloatParam(h, sim.shapefloatparam_init_bbox_max)
        return f" bbox={tuple(round(v,3) for v in bmin)}..{tuple(round(v,3) for v in bmax)}"
    except Exception:
        return ""

all_objs = sim.getObjectsInTree(sim.handle_scene, sim.handle_all)

# 建立 名称 -> 句柄 映射（可能有重名，取第一个）
name2h = {}
for o in all_objs:
    name2h.setdefault(sim.getObjectName(o), o)

# 建立 父句柄 -> 子句柄列表
children = {}
for o in all_objs:
    p = sim.getObjectParent(o)
    children.setdefault(p, []).append(o)

lines = []
def dump(o, depth=0):
    name = sim.getObjectName(o)
    pos = sim.getObjectPosition(o, -1)
    lines.append(f"{'  '*depth}- {name} [{typestr(o)}] pos={tuple(round(v,3) for v in pos)}{bbox_str(o)}")
    for c in sorted(children.get(o, []), key=lambda x: sim.getObjectName(x)):
        dump(c, depth+1)

lines.append("=" * 70)
lines.append("当前场景对象树")
lines.append("=" * 70)
lines.append(f"总对象数: {len(all_objs)}")
lines.append("")

for o in sorted(children.get(-1, []), key=lambda x: sim.getObjectName(x)):
    dump(o)

with open(OUT, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))
print(f"[OK] 已写出 {OUT}")
