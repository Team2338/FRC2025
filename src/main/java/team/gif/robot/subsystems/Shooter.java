// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.ToFSensor;

public class Shooter extends SubsystemBase {
    private static VictorSPX shooter;
    private static VictorSPX indexer;
    private DigitalInput indexerSensor;
    private DigitalInput exitSensor;
    private static ToFSensor sensorLeft;
    private static ToFSensor sensorRight;


    public Shooter() {
        shooter = new VictorSPX(RobotMap.SHOOTER_MOTOR_ID);
        sensorLeft = new ToFSensor(RobotMap.REEF_LEFT_SENSOR_ID);
        sensorRight = new ToFSensor(RobotMap.REEF_RIGHT_SENSOR_ID);
        shooter.configFactoryDefault();
        shooter.setNeutralMode(NeutralMode.Coast);
        shooter.setInverted(true);

        indexer= new VictorSPX(RobotMap.INDEXER_MOTOR_ID);
        indexer.configFactoryDefault();
        indexer.setNeutralMode(NeutralMode.Coast);
        indexer.setInverted(true);

        indexerSensor = new DigitalInput(RobotMap.INDEXER_GP_SENSOR_PORT);
        exitSensor = new DigitalInput(RobotMap.EXIT_GP_SENSOR_PORT);

        // todo: remove once specific values are determined
        SmartDashboard.putNumber(RobotMap.UI.SHOOTER_PERC, Constants.Shooter.SHOOT_PERCENT);
        SmartDashboard.putNumber(RobotMap.UI.INDEXER_PERC, Constants.Shooter.INDEX_PERCENT);
        SmartDashboard.putNumber(RobotMap.UI.STAGE_PERC, Constants.Shooter.STAGE_PERCENT);
    }

    /**
     * runs the shooter motor at a given power percentage
     * @param percent the percentage of power to apply to the motor
     **/
    public void runShooterMotor(double percent) {
        shooter.set(VictorSPXControlMode.PercentOutput, percent);
    }

    /**
     * runs the shooter motor at a power percentage determined by value on dashboard
     **/
    public void runShooterMotor() {
        // todo: change once value is determined
        //runShooterMotor(Constants.Shooter.SHOOT_PERCENT);
        runShooterMotor(SmartDashboard.getNumber(RobotMap.UI.SHOOTER_PERC, 0));
    }

    /**
     * runs the shooter motor at a power percentage determined by value on dashboard
     **/
    public void stageShooterMotor() {
        // todo: change once value is determined
        //runShooterMotor(Constants.Shooter.STAGE_PERCENT);
        runShooterMotor(SmartDashboard.getNumber(RobotMap.UI.STAGE_PERC, 0));
    }

    /**
     * stops the shooter motor
     **/
    public void stopShooterMotor() {
        runShooterMotor(0);
    }

    public double getLeftD() {
        return sensorLeft.getDistance();
    }

    public double getRightD() {
        return sensorRight.getDistance();
    }

    /**
     * returns true when both sensors are active
     * indicating robot is properly aligned with reef branch
     **/
    public boolean isShooterAligned() {
        return (sensorLeftActive() && sensorRightActive());
    }

    public boolean sensorLeftActive() {
        double sensorDistance = sensorLeft.getDistance();
        return sensorDistance < Constants.Shooter.REEF_SENSOR_TARGET_DISTANCE_MM && sensorDistance > 0;
    }

    public boolean sensorRightActive() {
        double sensorDistance = sensorRight.getDistance();
        return sensorDistance < Constants.Shooter.REEF_SENSOR_TARGET_DISTANCE_MM && sensorDistance > 0;
    }

    /**
     * runs the indexer motor at a given power percentage
     * @param percent the percentage of power to apply to the motor
     **/
    public void runIndexerMotor(double percent) {
        indexer.set(VictorSPXControlMode.PercentOutput, percent);
    }

    /**
     * runs the indexer motor at a power percentage determined by value on dashboard
     **/
    public void runIndexerMotor() {
        // todo: change once value is determined
        //runIndexerMotor(Constants.Shooter.INDEX_PERCENT);
        runIndexerMotor(SmartDashboard.getNumber(RobotMap.UI.INDEXER_PERC, 0));
    }

    /**
     * Stops the indexer motor
     **/
    public void stopIndexerMotor() {
        runIndexerMotor(0);
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
