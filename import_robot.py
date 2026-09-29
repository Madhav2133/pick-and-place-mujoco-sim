import mujoco_menagerie as mm
import mujoco
import matplotlib.pyplot as plt

model = mm.load('franka_emika_panda')
data = mujoco.MjData(model)

mujoco.mj_forward(model, data)

with mujoco.Renderer(model, height=480, width=640) as renderer:
    renderer.update_scene(data)
    img = renderer.render()

plt.imshow(img)
plt.axis("off")
plt.tight_layout()
plt.show()
