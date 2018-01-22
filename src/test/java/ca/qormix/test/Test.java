package ca.qormix.test;

import ca._4976.motion.Robot;
import ca._4976.motion.subsystems.Drive;
import ca._4976.motion.subsystems.Motion;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.HLUsageReporting;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Test {

    public static double output = 0;

    public static void main(String... args) throws InterruptedException {

        HLUsageReporting.SetImplementation(new HLUsageReporting.Interface() {
            @Override public void reportScheduler() { }
            @Override public void reportPIDController(int num) { }
            @Override public void reportSmartDashboard() { }
        });

        NetworkTableInstance.getDefault().setServer("localhost");
        NetworkTableInstance.getDefault().startServer();

        Motion motion = Robot.motion;
        Drive drive = Robot.drive;

        SmartDashboard.putData(drive);
        SmartDashboard.putData(motion);
        SmartDashboard.putData(new DriveTester());

        drive.enableRamping(true);
        motion.record();

        Thread.sleep(50);

        drive.arcadeDrive(0, 1);

        Thread.sleep(1500);

        drive.arcadeDrive(0, 0);

        Thread.sleep(1500);

        drive.arcadeDrive(0, -1);

        Thread.sleep(1500);

        drive.arcadeDrive(0, 0);

        Thread.sleep(1500);

        drive.enableRamping(false);
        motion.stop();

        Thread.sleep(50);

        drive.setUserControlEnabled(false);
        while (!Thread.interrupted()) {

            if (!motion.isRunning()) motion.run();

            Thread.sleep(200);
        }
    }

    static class DriveTester implements Sendable {

        @Override public String getName() { return "Drive Test"; }

        @Override public void setName(String name) { }

        @Override public String getSubsystem() { return null; }

        @Override public void setSubsystem(String subsystem) { }

        @Override public void initSendable(SendableBuilder builder) {

            builder.setSmartDashboardType("differentialDrive");

            builder.addDoubleProperty("Output", () -> output, it -> output = it);
        }
    }
}
