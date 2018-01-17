package ca._4976.motion.subsystems;

import ca._4976.motion.commands.DriveWithJoystick;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

import static ca.qormix.library.Lazy.using;

/**
 * The DriveTrain subsystem controls the robot's chassis and reads in
 * information about it's speed and posit ion.
 */
public class Drive extends Subsystem implements Runnable {

    private VictorSP leftFront = new VictorSP(0);
    private VictorSP leftRear = new VictorSP(1);
    private VictorSP rightFront = new VictorSP(2);
    private VictorSP rightRear = new VictorSP(3);

    private Encoder left = new Encoder(0, 1);
    private Encoder right = new Encoder(2, 3);

    private double[] ramp = new double[] { 1 / 200, 0.1 / 200 }; // (change per second / ticks per second) { max accel, max jerk }
    private double[] target = { 0, 0 };
    private double[] velocity = { 0, 0 };
    private double[] acceleration = { 0, 0 };

    private boolean ramping = false;
    private boolean userControlEnabled = true;

    @Override protected void initDefaultCommand() { setDefaultCommand(new DriveWithJoystick()); }

    public void setUserControlEnabled(boolean enabled) {

        if (!enabled) ramping = false;
        userControlEnabled = enabled;
    }

    public void stop() {

        leftFront.set(0);
        leftRear.set(0);
        rightFront.set(0);
        rightRear.set(0);
    }

    public void arcadeDrive(Joystick joy) {

        if (userControlEnabled) {

            double forward = joy.getRawAxis(3) - joy.getRawAxis(2);

            // Saves the joystick value as a power of 2 while still keeping the sign
            double turn = using(joy.getRawAxis(0), x -> x = x * x * (Math.abs(x) / x));

            target[0] = forward + turn;
            target[1] = -forward + turn;

            if (!ramping) setTankDrive(target[0], target[1]);
        }
    }

    @Override public void run() {

        double timing = 1e+9 / 200;
        long lastTick = System.nanoTime() - (long) timing;

        while (ramping) {

            if (System.nanoTime() - lastTick >= timing) {

                lastTick = System.nanoTime();

                for (int i = 0; i < 2; i ++) {

                    if (Math.abs(velocity[i] - target[i]) < ramp[1]) velocity[i] = target[i];

                    if (Math.abs(velocity[i] - target[i]) > 0) {


                        if (Math.abs(acceleration[i]) < ramp[0] 
                                && Math.abs((target[0] - velocity[0]) / acceleration[0]) > Math.abs(acceleration[i]) / ramp[1])
                            acceleration[i] += target[i] > velocity[i] ? ramp[1] : -ramp[1];

                        if (Math.abs((target[i] - velocity[i]) / acceleration[i]) < Math.abs(acceleration[i]) / ramp[1])
                            acceleration[i] -= target[i] > velocity[i] ? ramp[1] : -ramp[1];


                        velocity[i] += acceleration[i];

                    } else acceleration[i] = 0;
                }

                setTankDrive(velocity[0], velocity[1]);
            }
        }
    }

    public Double[] getEncoderPosition() { return new Double[] { left.getDistance(), right.getDistance() }; }

    public Double[] getEncoderRate() { return new Double[] { left.getRate(), right.getRate() }; }

    public boolean isStopped() { return left.getStopped() && right.getStopped(); }

    public synchronized void setTankDrive(double left, double right) {

        leftFront.set(left);
        leftRear.set(left);
        rightFront.set(right);
        rightRear.set(right);
    }

    public synchronized Double[] getTankDrive() { return new Double[] { leftFront.get(), rightFront.get() }; }

    public synchronized void enableRamping(boolean enable) {

        ramping = enable;

        if (enable) new Thread(this).start();
    }
}
