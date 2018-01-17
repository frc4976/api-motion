package ca._4976.motion;

import ca._4976.motion.commands.RecordProfile;
import ca._4976.motion.commands.RunProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * The operator interface of the robot, it has been simplified from the real
 * robot to allow control with a single PS3 joystick. As a result, not all
 * functionality from the real robot is available.
 */
public final class OI {

    public Joystick driver = new Joystick(0);
    public Joystick operator = new Joystick(1);

    OI() {

        new JoystickButton(operator, 7).whenPressed(new RecordProfile());
        new JoystickButton(operator, 8).whenPressed(new RunProfile());
    }
}
