import time
import numpy as np
import mujoco
import mujoco.viewer

from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_DIR / "assets" / "robot_arm.xml"   # save the XML here

# Set to True to see the arm sweep through its full range automatically
DEMO_MOTION = False


def main():

    # ------------------------------------------------------------------
    # Load robot model
    # ------------------------------------------------------------------
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    # ------------------------------------------------------------------
    # Joint addresses  (names match the XML)
    # ------------------------------------------------------------------
    joint1_id = model.joint("theta1").id      # revolute  (rotary base)
    joint2_id = model.joint("d2").id          # prismatic (linear slide)

    qpos1 = model.jnt_qposadr[joint1_id]
    qpos2 = model.jnt_qposadr[joint2_id]

    # ------------------------------------------------------------------
    # Actuator addresses
    # ------------------------------------------------------------------
    motor1 = model.actuator("theta1_act").id
    motor2 = model.actuator("d2_act").id

    # Control limits straight from the XML (used to clamp commands)
    lo1, hi1 = model.actuator_ctrlrange[motor1]
    lo2, hi2 = model.actuator_ctrlrange[motor2]

    # Tool site
    tool_site = model.site("tool_site").id

    # ------------------------------------------------------------------
    # Initial joint configuration
    # ------------------------------------------------------------------
    angle = np.deg2rad(45)
    extension = 0.40

    data.qpos[qpos1] = angle
    data.qpos[qpos2] = extension

    data.ctrl[motor1] = angle
    data.ctrl[motor2] = extension

    # Calculate initial robot positions
    mujoco.mj_forward(model, data)

    print("\nRobot loaded successfully!")
    print("Joints:", model.njnt)
    print("Actuators:", model.nu)

    print("\nInitial Configuration")
    print("---------------------")
    print("theta1 :", np.rad2deg(angle), "degrees")
    print("d2     :", extension, "meters")
    print("Tool   :", data.site_xpos[tool_site].copy())

    # ------------------------------------------------------------------
    # Open MuJoCo viewer
    # ------------------------------------------------------------------
    with mujoco.viewer.launch_passive(model, data) as viewer:

        # Camera configuration
        viewer.cam.lookat[:] = [0.25, 0.0, 0.12]
        viewer.cam.distance = 1.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

        t0 = time.time()

        while viewer.is_running():
            step_start = time.time()

            # Optional demo: sweep both joints
            if DEMO_MOTION:
                t = time.time() - t0
                data.ctrl[motor1] = np.clip(
                    np.pi * (1 + np.sin(0.4 * t - np.pi / 2)), lo1, hi1)
                data.ctrl[motor2] = np.clip(
                    0.2 * (1 + np.sin(0.8 * t - np.pi / 2)), lo2, hi2)

            mujoco.mj_step(model, data)
            viewer.sync()

            # Keep simulation close to real time
            elapsed = time.time() - step_start
            sleep_time = model.opt.timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    main()