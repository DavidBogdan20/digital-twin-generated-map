"""Spawns a static cone on a grid at a height and lists which are dormant right away (a tile the large map manager
thinks is not loaded). usage: dormant_grid.py x0 x1 y0 y1 step z"""
import sys
import carla
x0, x1, y0, y1, step, z = map(float, sys.argv[1:7])
c = carla.Client("localhost", 2000); c.set_timeout(60)
w = c.get_world()
s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05; w.apply_settings(s)
bp = w.get_blueprint_library().find("static.prop.constructioncone")
cmds, pts = [], []
x = x0
while x <= x1:
    y = y0
    while y <= y1:
        cmds.append(carla.command.SpawnActor(bp, carla.Transform(carla.Location(x, y, z)))); pts.append((x, y)); y += step
    x += step
res = c.apply_batch_sync(cmds, True)
ids = [(r.actor_id, p) for r, p in zip(res, pts) if not r.error]
for _ in range(3): w.tick()
actors = {a.id: a for a in w.get_actors([i for i, _ in ids])}
dormant = sorted(p for i, p in ids if actors[i].is_dormant)
print("spawned", len(ids), "dormant", len(dormant))
xs = sorted({p[0] for p in dormant}); ys = sorted({p[1] for p in dormant})
if dormant:
    print("dormant x range", xs[0], xs[-1], "y range", ys[0], ys[-1])
    print(dormant[:20])
c.apply_batch_sync([carla.command.DestroyActor(i) for i, _ in ids], True)
s.synchronous_mode = False; s.fixed_delta_seconds = None; w.apply_settings(s)
