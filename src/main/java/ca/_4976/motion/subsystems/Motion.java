package ca._4976.motion.subsystems;

import ca._4976.motion.commands.ListenableCommand;
import ca._4976.motion.Robot;
import ca._4976.motion.commands.SaveProfile;
import ca._4976.motion.data.Moment;
import ca._4976.motion.data.Profile;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import static ca.qormix.library.Lazy.use;

/**
 * The Motion subsystem controls records and plays back saved
 * information about the chassis speed and position.
 */
public final class Motion extends Subsystem {

    private DriverStation ds = DriverStation.getInstance();
    private Profile profile = Profile.blank();
    private Drive drive = Robot.drive;
    private boolean isRunning = false;
    private boolean isRecording = false;

    public ListenableCommand[] commands = new ListenableCommand[0];
    public ArrayList<Integer> report = new ArrayList<>();

    private double p, i, d;

    @Override protected void initDefaultCommand() { }

    public Motion() {

        use(NetworkTableInstance.getDefault().getTable("Motion"), it -> {

            NetworkTableEntry tableEntry = it.getEntry("PID");

            double[] pid = { 0, 0, 0 };

            if (!tableEntry.exists()) {

               tableEntry.setDoubleArray(pid);
               tableEntry.setPersistent();
            }

            p = pid[0];
            i = pid[1];
            d = pid[2];

            it.addEntryListener((table, key, entry, value, flags) -> {

                double[] val = tableEntry.getDoubleArray(new double[] { 0, 0, 0 });

                p = val[0];
                i = val[1];
                d = val[2];

           }, 0);
        });
    }

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


            while (isRunning && ds.isEnabled()) {

                if (System.nanoTime() - lastTick >= timing) {

                    lastTick = System.nanoTime();

                    final Moment moment = profile.moments[interval];

                    use(drive.getEncoderPosition(), it -> {

                        error[0] = moment.position[0] - it[0];
                        error[1] = moment.position[1] - it[1];
                    });

                    integral[0] += error[0];
                    integral[1] += error[1];

                    use(drive.getEncoderRate(), it -> {

                        derivative[0] = (error[0] - lastError[0]) / (1/200) - it[0];
                        derivative[1] = (error[1] - lastError[1]) / (1/200) - it[1];
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
        }
    }
}
