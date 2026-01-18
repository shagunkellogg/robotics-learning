import time
import numpy as np
import mujoco
import mujoco.viewer

XML = """
<mujoco model="one_joint_arm">
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <worldbody>
    <light pos="0 0 3"/>

    <!-- Base -->
    <body name="base" pos="0 0 0.5">
      <geom type="box" size="0.08 0.08 0.08" rgba="0.3 0.3 0.3 1"/>

      <!-- Arm -->
      <body name="link1" pos="0 0 0">
        <joint name="hinge" type="hinge" axis="0 1 0" range="-90 90"/>
        <geom type="capsule" fromto="0 0 0  0.35 0 0" size="0.03" rgba="0.2 0.6 0.9 1"/>

        <!-- End effector -->
        <body name="tip" pos="0.35 0 0">
          <geom type="sphere" size="0.03" rgba="0.9 0.3 0.3 1"/>
        </body>
      </body>
    </body>

    <!-- Target marker -->
    <body name="target" pos="0.3 0 0.7">
      <geom type="sphere" size="0.03" rgba="0.1 0.9 0.1 1"/>
    </body>
  </worldbody>

  <actuator>
    <position joint="hinge" kp="40"/>
  </actuator>
</mujoco>
"""

def main():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    tip_id = model.body("tip").id
    target_id = model.body("target").id
    hinge_qpos_adr = model.jnt_qposadr[model.joint("hinge").id]
    hinge_range = model.jnt_range[model.joint("hinge").id]  # degrees in XML, radians in qpos

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():

            # Get tip & target positions (world frame)
            tip_pos = data.xpos[tip_id]
            target_pos = data.xpos[target_id]

            # Vector from tip to target (world frame)
            error = target_pos - tip_pos

            # Compute translational Jacobian of the tip position wrt joint velocities
            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jacBody(model, data, jacp, jacr, tip_id)

            # For a 1-DOF arm, use Jacobian transpose to convert XYZ error -> joint delta
            k = 2.0  # gain; increase if too slow, decrease if unstable
            dq = k * float(jacp[:, 0].dot(error))

            # Position actuator expects a desired joint position, not an increment
            q_des = float(data.qpos[hinge_qpos_adr] + dq)

            # Clamp to joint limits (MuJoCo stores joint ranges in radians)
            q_min = float(hinge_range[0])
            q_max = float(hinge_range[1])
            q_des = float(np.clip(q_des, q_min, q_max))

            data.ctrl[0] = q_des

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

if __name__ == "__main__":
    main()