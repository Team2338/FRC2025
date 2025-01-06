package team.gif.robot.subsystems.drivers;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Pigeon {

    public static PigeonIMU _pigeon;

    private final PigeonIMU.GeneralStatus _pigeonGenStatus = new PigeonIMU.GeneralStatus();

    private double pitchOffset = 0;

    public Pigeon(int PigeonID) {
        _pigeon = new PigeonIMU(PigeonID);
    }

    public Pigeon(TalonSRX talon) {
        _pigeon = new PigeonIMU(talon);
    }

    public void addToShuffleboard(String tabName, String widgetTitle) {
        // Puts a Gyro type widget on dashboard and assigns
        // the function getHeading_Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab(tabName); //gets a reference to the shuffleboard tab
        tab.add(widgetTitle, (x) -> {
            x.setSmartDashboardType("Gyro");
            x.addDoubleProperty("Value", () -> getCompassHeading(), null);
        });
    }

    /**
     * Returns heading from pigeon
     * turning counterclockwise, values increase
     * turning clockwise, values decrease
     * no rollover, can go negative
     * valid range is 23040 to -23040
     */
    public double getHeading() {
        double[] ypr = new double[3];

        _pigeon.getYawPitchRoll(ypr);

        return ypr[0] / (7216.171875 / 7200); // added fudge factor, rotated the robot 20 times, yaw read 7216, should have read 7200.
    }

    /**
     * Returns Rotation2d object using heading from the pigeon
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(Units.degreesToRadians(getHeading()));
    }

    /**
     * Returns heading values between -180 and 180
     * Does this by using the IEEEremainder function which
     * splits the remainder in half and returns anything greater than
     * half as negative.
     */
    public double get180Heading() {
        return Math.IEEEremainder(getHeading(), 360.0d);
    }

    /**
     * Returns heading from pigeon
     * from 0 to 359.99 turning counterclockwise
     */
    public double get360Heading() {
        double heading = getHeading(); // Returns heading from 23040 to -23040

        // Need to convert the heading to a value between 0 and 360
        heading = heading < 0 ? 360 + (heading % 360) : heading % 360;

        return heading;
    }

    /**
     * The heading value from the pigeon increases counterclockwise (0 North, 90 West, 180 South, 270 East)
     * Some features need degrees to look like a compass,
     * increasing clockwise (0 North, 90 East, 180 South, 270 West)
     * with rollover (max value 359.99, min 0)
     * <p>
     * Returns heading from pigeon
     * from 0 to 359.99 turning clockwise
     */
    public double getCompassHeading() {
        double heading = getHeading();

        // Get the heading. If the value is negative, need to flip it positive
        // also need to subtract from 360 to flip value to the correct compass heading
        heading = heading < 0 ? (0 - heading % 360) : (360 - (heading % 360));

        // 0 degree will result in 360, so set it back to 0
        heading = heading == 360.0 ? 0 : heading;

        return heading;
    }

    public boolean isActive() {
        _pigeon.getGeneralStatus(_pigeonGenStatus);

        return _pigeonGenStatus.state == PigeonIMU.PigeonState.Ready;
    }

    public void resetPigeonPosition() {
//        System.out.println("resetting pigeon empty"); // for debugging
        resetPigeonPosition(0);
    }

    /**
     * Reset the pigeon position to something other than 0
     * @param angle the initial angle in degrees
     */
    public void resetPigeonPosition(double angle) {
//        System.out.println("resetting pigeon " + angle); // for debugging
        setYaw(angle);
        zeroPitch();
    }

    public void setYaw(double yaw) {
        _pigeon.setYaw(yaw);
    }

    public void zeroPitch() {
        pitchOffset = getYPR()[1];
    }

    public double[] getYPR() {
        double[] ypr = new double[3];
        _pigeon.getYawPitchRoll(ypr);
//        System.out.format("YPR %.1f %.1f %.1f", ypr[0], ypr[1], ypr[2]); // for debugging
        return ypr;
    }

    public double[] getQuaternions() {
        // Quaternions
        double[] quaternions = new double[4];
        _pigeon.get6dQuaternion(quaternions);
        return quaternions;
    }

    public double getTemp() {
        return _pigeon.getTemp();
    }

    public double[] getAccumulatedGyro() {
        double[] accumGyro = new double[3];
        _pigeon.getAccumGyro(accumGyro);
        return accumGyro;
    }

    public short[] getBiasedAccel() {
        short[] biasedAccel = new short[3];
        _pigeon.getBiasedAccelerometer(biasedAccel);
        return biasedAccel;
    }

    public double[] getRawGyro() {
        double[] rawGyro = new double[3];
        _pigeon.getRawGyro(rawGyro);
        return rawGyro;
    } // Angular velocities

    public double[] getAccelAngles() {
        double[] accelAngles = new double[3];
        _pigeon.getAccelerometerAngles(accelAngles);
        return accelAngles;
    }

    public short[] getBiasedMagnetometer() {
        short[] biasedMagnet = new short[3];
        _pigeon.getBiasedMagnetometer(biasedMagnet);
        return biasedMagnet;
    }

    // Raw magnetometer
    public short[] getRawMagnet() {
        short[] rawMagnet = new short[3];
        _pigeon.getRawMagnetometer(rawMagnet);
        return rawMagnet;
    }

    public double getPitch() {
        double[] ypr = new double[3];

        _pigeon.getYawPitchRoll(ypr);

        return ypr[1] - pitchOffset;
    }
}
