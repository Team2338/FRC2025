// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private static TalonSRX shooter;
    private static TalonSRX indexer;
    private DigitalInput indexerSensor;
    private DigitalInput exitSensor;


    public Shooter() {
        shooter = new TalonSRX(RobotMap.SHOOTER_MOTOR_ID);
        shooter.configFactoryDefault();
        shooter.setNeutralMode(NeutralMode.Coast);
        shooter.setInverted(true);

        indexer= new TalonSRX(RobotMap.INDEXER_MOTOR_ID);
        indexer.configFactoryDefault();
        indexer.setNeutralMode(NeutralMode.Coast);

        indexerSensor = new DigitalInput(RobotMap.INDEXER_SENSOR_ID);
        exitSensor = new DigitalInput(RobotMap.EXIT_SENSOR_ID);

        SmartDashboard.putNumber(RobotMap.UI.SHOOTER_PERC, 1.0);
        SmartDashboard.putNumber(RobotMap.UI.INDEXER_PERC, 0.35);
        SmartDashboard.putNumber(RobotMap.UI.STAGE_PERC, 0.20);
    }

    /**
     * runs the shooter motor at a given power percentage
     * @param percent the percentage of power to apply to the motor
     **/
    public void runShooterMotor(double percent) {
        shooter.set(TalonSRXControlMode.PercentOutput, percent);
    }
//
//    public void moveIndexerFromShuffleboard() {
//        indexer.set(TalonSRXControlMode.PercentOutput, SmartDashboard.getNumber(RobotMap.UI.INDEXER_PERC, 0));
//    }

    /**
     * runs the shooter motor at a power percentage determined by value on dashboard
     **/
    public void runShooterMotor() {
        runShooterMotor(SmartDashboard.getNumber(RobotMap.UI.SHOOTER_PERC, 0));
    }

    public void stopShooterMotor() {
        runShooterMotor(0);
    }

    /**
     * runs the indexer motor at a given power percentage
     * @param percent the percentage of power to apply to the motor
     **/
    public void runIndexerMotor(double percent) {
        indexer.set(TalonSRXControlMode.PercentOutput, percent);
    }

    /**
     * runs the indexer motor at a power percentage determined by value on dashboard
     **/
    public void runIndexerMotor() {
        runIndexerMotor(SmartDashboard.getNumber(RobotMap.UI.INDEXER_PERC, 0));
    }

    /**
     * temporary method to run stage from value in dashboard
     * @return value entered by user in dashboard
     */
    public double getStagePercent() {
        return SmartDashboard.getNumber(RobotMap.UI.STAGE_PERC, 0);
    }

    public boolean getIndexerSensorState() {
        return indexerSensor.get();
    }

    public boolean getExitSensorState() {
        return exitSensor.get();
    }

    public void setShooterBrakeMode() {
        shooter.setNeutralMode(NeutralMode.Brake);
    }

    public void setShooterCoastMode() {
        shooter.setNeutralMode(NeutralMode.Coast);
    }
}
