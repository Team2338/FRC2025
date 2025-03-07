// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Robot;

public class Diagnostics extends SubsystemBase {

    private static int flashCounter;

    public Diagnostics() {
        flashCounter = 0;
    }

    public boolean getDriveMotorTempHot() {
        return (Robot.swerveDrive.fL.isDriveMotorHot() ||
                Robot.swerveDrive.fR.isDriveMotorHot() ||
                Robot.swerveDrive.rL.isDriveMotorHot() ||
                Robot.swerveDrive.rR.isDriveMotorHot());
    }
    
    public boolean getAnyMotorTempHot() {
        return (getDriveMotorTempHot() ||
                Robot.elevator.isElevatorMotorHot());
    }

    public boolean getAnyMotorTempHotFlash() {
        final int FLASH_CYCLES = 2; //number of 500ms intervals in a period

        // if all motors cool return false
        if (!getAnyMotorTempHot()) {
            return false;
        }
        // if at least 1 motor hot, flash with 50% duty cycle
        return ++flashCounter % FLASH_CYCLES < FLASH_CYCLES/2;
    }

    public boolean getAtTargetAngle() {
        double angle = Robot.pigeon.getCompassHeading();
        double tolerance = 3.0;
        boolean result = false;

        if (angle <= 3 || angle >= 357) {
            result = true;
        }

        if (Math.abs(angle - 135) <= tolerance) {
            result = true;
        }

        if (Math.abs(angle - 180) <= tolerance) {
            result = true;
        }

        if (Math.abs(angle - 225) <= tolerance) {
            result = true;
        }

        if (Math.abs(angle - 270) <= tolerance) {
            result = true;
        }

        return result;
    }
}

