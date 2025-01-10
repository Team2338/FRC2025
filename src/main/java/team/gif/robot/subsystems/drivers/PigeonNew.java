// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems.drivers;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonNew extends SubsystemBase {

    public static Pigeon2 _pigeon;
    private double pitchOffset = 0;
    public PigeonNew(int PigeonID) {
        _pigeon = new Pigeon2(PigeonID);

    }
    public void addToShuffleboard(String tabName, String widgetTitle) {

        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        tab.add(widgetTitle, (x) -> {
            x.setSmartDashboardType("Gyro");
            x.addDoubleProperty("Value", () -> getCompassHeading(), null);
        });
    }





    public double getCompassHeading() {
        double heading = getHeading();

        // Get the heading. If the value is negative, need to flip it positive
        // also need to subtract from 360 to flip value to the correct compass heading
        heading = heading < 0 ? (0 - heading % 360) : (360 - (heading % 360));

        // 0 degree will result in 360, so set it back to 0
        heading = heading == 360.0 ? 0 : heading;

        return heading;
    }


    public double getHeading() {
        double[] ypr = new double[3];

        ypr[0] = _pigeon.getYaw().getValueAsDouble();
        ypr[1] = _pigeon.getPitch().getValueAsDouble();
        ypr[2] = _pigeon.getRoll().getValueAsDouble();

        return ypr[0];
    }

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
        ypr[0] = _pigeon.getPitch().getValueAsDouble();
        ypr[0] = _pigeon.getRoll().getValueAsDouble();
//        System.out.format("YPR %.1f %.1f %.1f", ypr[0], ypr[1], ypr[2]); // for debugging
        return ypr;
    }

    public void resetPigeonPosition() {
//        System.out.println("resetting pigeon empty"); // for debugging
        resetPigeonPosition(0);
    }
    public void resetPigeonPosition(double angle) {
//        System.out.println("resetting pigeon " + angle); // for debugging
        setYaw(angle);
        zeroPitch();
    }
}
