import mujoco
import numpy as np
import matplotlib.pyplot as plt
import mujoco_menagerie as mm

HOME = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]

def build_scene():

    panda_m = mm.load('franka_emika_panda')
    spec = mm.get('franka_emika_panda').spec()
    world = spec.worldbody

    world.add_light(pos=[0, 0, 3], dir=[0, 0, -1])

    # --- Table: slab plus four legs, robot base sits at the origin ---
    TABLE_H = 0.30                      # tabletop height (z)
    TABLE_CENTER = np.array([0.60, 0.0])
    TABLE_HALF = np.array([0.30, 0.40])  # half extents in x, y
    SLAB_T = 0.02                        # slab half thickness

    world.add_geom(
        name="table_top", type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[*TABLE_CENTER, TABLE_H - SLAB_T],
        size=[*TABLE_HALF, SLAB_T],
        rgba=[0.55, 0.38, 0.22, 1],
    )
    leg_h = (TABLE_H - 2 * SLAB_T) / 2
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        world.add_geom(
            name=f"table_leg_{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[TABLE_CENTER[0] + sx * (TABLE_HALF[0] - 0.03),
                TABLE_CENTER[1] + sy * (TABLE_HALF[1] - 0.03),
                leg_h],
            size=[0.02, 0.02, leg_h],
            rgba=[0.4, 0.28, 0.16, 1],
        )


    # --- Cube to pick (free body so it can move) ---
    CUBE_HALF = 0.02
    cube = world.add_body(name="cube", pos=[0.50, -0.15, TABLE_H + CUBE_HALF])
    cube.add_freejoint()
    cube.add_geom(
        name="cube_geom", type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[CUBE_HALF] * 3, mass=0.05,
        rgba=[0.9, 0.15, 0.15, 1],
        friction=[1.0, 0.005, 0.0001],
    )

    # --- Target marker (visual only, no collisions) ---
    world.add_geom(
        name="target", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0.50, 0.20, TABLE_H + 0.001],
        size=[0.04, 0.001, 0],
        rgba=[0.1, 0.8, 0.2, 0.7],
        contype=0, conaffinity=0,
    )


    model = spec.compile()
    data = mujoco.MjData(model)

    return model, data

# # Put the arm in a sensible "ready" pose (Panda home configuration)
# data.qpos[:7] = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]
# mujoco.mj_forward(model, data)

# # --- Camera and render ---
# cam = mujoco.MjvCamera()
# cam.lookat[:] = [0.4, 0.0, 0.35]
# cam.distance = 1.6
# cam.azimuth = 135
# cam.elevation = -25

# with mujoco.Renderer(model, height=480, width=640) as renderer:
#     renderer.update_scene(data, camera=cam)
#     img = renderer.render()

# plt.imshow(img)
# plt.axis("off")
# plt.tight_layout()
# plt.show()