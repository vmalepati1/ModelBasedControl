package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.JoystickSim;

import static frc.robot.Constants.InputDevices.LEFT_JOYSTICK_PORT;
import static frc.robot.Constants.InputDevices.RIGHT_JOYSTICK_PORT;

public class OI {

    public static final Joystick leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
    public static final Joystick rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);

    public static final JoystickSim leftJoystickSim = new JoystickSim(leftJoystick);
    public static final JoystickSim rightJoystickSim = new JoystickSim(rightJoystick);

}
