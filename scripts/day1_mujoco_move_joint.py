import time
import math
import mujoco
import mujoco.viewer

# Minimal 1-joint arm (MuJoCo XML)
XML = """
<mujoco model="one_joint_arm">
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <worldbody>
    <light pos="0 0 3"/>

    <body name="base" pos="0 0 0.5">
      <geom type="box" size="0.08 0.08 0.08" rgba="0.3 0.3 0.3 1"/>

      <body name="link1" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0" range="-90 90" damping="1"/>
        <geom type="capsule" fromto="0 0 0  0.35 0 0" size="0.03" rgba="0.2 0.6 0.9 1"/>

        <body name="tip" pos="0.35 0 0">
          <geom type="sphere" size="0.035" rgba="0.9 0.3 0.3 1"/>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position joint="hinge" kp="30"/>
  </actuator>
</mujoco>
"""

def main():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        while viewer.is_running():
            t = time.time() - t0
            target = 0.9 * math.sin(1.5 * t)  # radians
            data.ctrl[0] = target

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

if __name__ == "__main__":
    main()