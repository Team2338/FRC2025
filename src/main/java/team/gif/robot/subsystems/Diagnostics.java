// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class Diagnostics extends SubsystemBase {

    static int flashCounter;

    public Diagnostics() {
        flashCounter = 0;
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
        final int FLASH_PERIOD_CYCLES = 40; // number of 20 msec cycles (50 = 1 sec)

        
        if (!getAnyMotorTempHot()) {
            // if the temps are all good, just return false
            return false;
        } else {
            // if the temps are hot, use a 50% duty cycle to flash between true/false over FLASH_PERIOD_CYCLES
            ++flashCounter;
            return flashCounter % FLASH_PERIOD_CYCLES < (FLASH_PERIOD_CYCLES/2);
        }
    }
}
