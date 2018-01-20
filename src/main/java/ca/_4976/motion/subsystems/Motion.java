package ca._4976.motion.subsystems;

import ca._4976.motion.commands.ListenableCommand;
import ca._4976.motion.Robot;
import ca._4976.motion.commands.SaveProfile;
import ca._4976.motion.data.Moment;
import ca._4976.motion.data.Profile;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import static ca.qormix.library.Lazy.use;

/**
 * The Motion subsystem controls records and plays back saved
 * information about the chassis speed and position.
 */
public final class Motion extends Subsystem implements Sendable {

    private DriverStation ds = DriverStation.getInstance();
    private Profile profile = Profile.blank();
    private Drive drive = Robot.drive;
    private boolean isRunning = false;
    private boolean isRecording = false;

    public ListenableCommand[] commands = new ListenableCommand[0];
    public ArrayList<Integer> report = new ArrayList<>();

    private double p = 0, i = 0, d = 0;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Motion");
    private final NetworkTableEntry leftError = table.getEntry("Left Error");
    private final NetworkTableEntry rightError = table.getEntry("Right Error");

    @Override protected void initDefaultCommand() { }

    public boolean isRecording() { return isRecording; }

    public boolean isRunning() { return isRunning; }

    public synchronized void record() { new Thread(new Record()).start(); }

    public synchronized void run() { new Thread(new Run()).start(); }

    public synchronized void stop() {

        isRunning = false;
        isRecording = false;
    }

    private class Record implements Runnable {

        @Override public void run() {

            isRecording = true;

            double timing = 1e+9 / 200;
            long lastTick = System.nanoTime() - (long) timing;

            ArrayList<Moment> moments = new ArrayList<>();

            while (isRecording && ds.isEnabled()) {

                if (System.nanoTime() - lastTick >= timing) {

                    lastTick = System.nanoTime();

                    moments.add(new Moment(
                            report.toArray(new Integer[report.size()]),
                            Robot.drive.getTankDrive(),
                            Robot.drive.getEncoderPosition(),
                            Robot.drive.getEncoderRate()
                    ));

                    report.clear();
                }
            }

            DateFormat dateFormat = new SimpleDateFormat("YYYY.MM.DD");

            profile = new Profile(
                    "Recording - " + System.currentTimeMillis() % 10000,
                    dateFormat.format(new Date()),
                    moments.toArray(new Moment[moments.size()])
            );

            new SaveProfile(profile).start();
            isRecording = false;
        }
    }

    private class Run implements Runnable {

        @Override public void run() {

            isRunning = true;

            double timing = 1e+9 / 200;
            long lastTick = System.nanoTime() - (long) timing;

            int interval = 0;

            double[] error = new double[2];
            double[] integral = new double[2];
            double[] derivative = new double[2];
            double[] lastError = new double[2];

            while (isRunning && ds.isEnabled() && interval < profile.moments.length) {

                if (System.nanoTime() - lastTick >= timing)  {

                    lastTick = System.nanoTime();

                    final Moment moment = profile.moments[interval];

                    use(drive.getEncoderPosition(), it -> {

                        error[0] = moment.position[0] - it[0];
                        error[1] = moment.position[1] - it[1];
                    });

                    leftError.setDouble(error[0]);
                    rightError.setDouble(error[1]);

                    integral[0] += error[0];
                    integral[1] += error[1];

                    use(drive.getEncoderRate(), it -> {

                        derivative[0] = (error[0] - lastError[0]) / (1.0/200) - it[0];
                        derivative[1] = (error[1] - lastError[1]) / (1.0/200) - it[1];
                    });

                    lastError[0] = error[0];
                    lastError[1] = error[1];

                    drive.setTankDrive(
                            moment.output[0]
                                + p * error[0]
                                + i * integral[0]
                                + d * derivative[0]
                            ,
                            moment.output[1]
                                + p * error[1]
                                + i * integral[1]
                                + d * derivative[1]
                    );

                    for (int command : moment.commands) commands[command].start();

                    interval++;
                }
            }

            isRunning = false;
        }
    }

    @Override public void initSendable(SendableBuilder builder) {

        setName("Motion Profile PID");

        builder.setSmartDashboardType("PIDController");
        builder.setSafeState(this::stop);
        builder.addDoubleProperty("p", () -> p, it -> p = it);
        builder.addDoubleProperty("i", () -> i, it -> i = it);
        builder.addDoubleProperty("d", () -> d, it -> d = it);
        builder.addBooleanProperty("enabled", this::isRunning, ignored -> {});
    }
}
