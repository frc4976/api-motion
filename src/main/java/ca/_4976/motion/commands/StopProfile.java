package ca._4976.motion.commands;

import ca._4976.motion.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This command will stop a recording/running profile.
 */
public class StopProfile extends Command {

    public StopProfile() {

        requires(Robot.motion);
        requires(Robot.drive);

        willRunWhenDisabled();
    }

    @Override protected void initialize() {

        Robot.motion.stop();
    }

    @Override protected boolean isFinished() { return !Robot.motion.isRunning(); }

    @Override protected void end() { Robot.drive.setUserControlEnabled(true); }
}
