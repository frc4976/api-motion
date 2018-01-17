package ca._4976.motion.commands;

import ca._4976.motion.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This command will begin running the profile that is saved in memory.
 */
public final class RunProfile extends Command {

    public RunProfile() {

        requires(Robot.motion);
        requires(Robot.drive);

        willRunWhenDisabled();
    }

    @Override protected void initialize() {

        Robot.drive.setUserControlEnabled(false);
        Robot.motion.run();
    }

    @Override protected boolean isFinished() { return !Robot.motion.isRunning(); }

    @Override protected void end() { Robot.drive.setUserControlEnabled(true); }
}
