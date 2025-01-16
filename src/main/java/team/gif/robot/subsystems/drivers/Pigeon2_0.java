// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems.drivers;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon2_0 extends SubsystemBase {

    public static Pigeon2 _pigeon;
    private double pitchOffset = 0;
    public Pigeon2_0(int PigeonID) {
        _pigeon = new Pigeon2(PigeonID);

    }
    public void addToShuffleboard(String tabName, String widgetTitle) {
        // Puts a Gyro type widget on dashboard and assigns
        // the function getHeading_Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        tab.add(widgetTitle, (x) -> {
            x.setSmartDashboardType("Gyro");
            x.addDoubleProperty("Value", this::getCompassHeading, null);
        });
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

    /**
     * Returns heading from pigeon
     * turning counterclockwise, values increase
     * turning clockwise, values decrease
     * no rollover, can go negative
     * valid range is 23040 to -23040
     */
    public double getHeading() {
        double[] ypr = new double[3];

        ypr[0] = _pigeon.getYaw().getValueAsDouble();
        ypr[0] = _pigeon.getPitch().getValueAsDouble();
        ypr[0] = _pigeon.getRoll().getValueAsDouble();

        return ypr[0];
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
     * Returns Rotation2d object using heading from the pigeon
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(Units.degreesToRadians(getHeading()));
    }

    public void setYaw(double yaw) {
        _pigeon.setYaw(yaw);
    }

    public void zeroPitch() {
        pitchOffset = getYPR()[1];
    }

    public double[] getYPR() {
        double[] ypr = new double[3];

        ypr[0] = _pigeon.getYaw().getValueAsDouble();
        ypr[1] = _pigeon.getPitch().getValueAsDouble();
        ypr[2] = _pigeon.getRoll().getValueAsDouble();
//        System.out.format("YPR %.1f %.1f %.1f", ypr[0], ypr[1], ypr[2]); // for debugging
        return ypr;
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

    public double getPitch() {
        return _pigeon.getPitch().getValueAsDouble();
    }
}
