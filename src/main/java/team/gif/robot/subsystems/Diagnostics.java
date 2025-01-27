// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class Diagnostics extends SubsystemBase {

    private int counter = 0;

    public Diagnostics() {
    }

    public boolean getDriveMotorTempHot() {
        return (Robot.swerveDrive.fLDriveTemp() >= Constants.MotorTemps.DRIVETRAIN_MOTOR_TEMP ||
                Robot.swerveDrive.fRDriveTemp() >= Constants.MotorTemps.DRIVETRAIN_MOTOR_TEMP ||
                Robot.swerveDrive.rLDriveTemp() >= Constants.MotorTemps.DRIVETRAIN_MOTOR_TEMP ||
                Robot.swerveDrive.rRDriveTemp() >= Constants.MotorTemps.DRIVETRAIN_MOTOR_TEMP);
    }

    public boolean getAnyMotorTempHot() {
        return getDriveMotorTempHot();
    }

    public boolean getAnyMotorTempHotFlash() {
        final int FLASH_CYCLE = 2; //number of 500ms intervals in a period

        // if all motors cool return false
        if (!getAnyMotorTempHot()) {
            return false;
        }
        // if at least 1 motor hot, flash with 50% duty cycle
        return ++counter % FLASH_CYCLE < FLASH_CYCLE/2;
    }
}

