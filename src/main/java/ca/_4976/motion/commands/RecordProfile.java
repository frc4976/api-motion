package ca._4976.motion.commands;

import ca._4976.motion.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This command allows the robot to start recording a new profile.
 * When complete the profile will be saved in memory ready to replay.
 */
public class RecordProfile extends Command {

    public RecordProfile() {

        requires(Robot.drive);
        requires(Robot.motion);

        willRunWhenDisabled();
    }

    @Override protected void initialize() {

        Robot.motion.record();
        Robot.drive.enableRamping(true);
    }

    @Override protected boolean isFinished() { return !Robot.motion.isRecording(); }
}
