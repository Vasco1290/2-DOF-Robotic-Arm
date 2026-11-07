import mujoco
import mujoco.viewer

MODEL_PATH = "assets/robot_arm.xml"

def main():

    #Load robot model
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)

    #Create simulation data
    data = mujoco.MjData(model)

    print("Robot loaded successfully!")
    print("Number of joints:",model.njnt)
    print("Number of actuators:", model.nu)

    #Open simulator viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running():

            mujoco.mj_step(model, data)

            viewer.sync()

if __name__ == "__main__":
    main()