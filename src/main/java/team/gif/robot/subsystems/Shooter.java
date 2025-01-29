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


    /** Creates a new ExampleSubsystem. */
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

        SmartDashboard.putNumber(RobotMap.UI.SHOOTER_PERC, 0.4);
        SmartDashboard.putNumber(RobotMap.UI.INDEXER_PERC, 0);
    }

    public void moveMotor(double percentOutput) {
        shooter.set(TalonSRXControlMode.PercentOutput, percentOutput);
    }

    public void moveIndexerFromShuffleboard() {
        indexer.set(TalonSRXControlMode.PercentOutput, 0.35);
    }

    public void moveFromShuffleboard() {
        moveMotor(SmartDashboard.getNumber(RobotMap.UI.SHOOTER_PERC, 0));
    }

    public boolean getIndexerSensorState() {
        return indexerSensor.get();
    }

    public boolean getExitSensorState() {
        return exitSensor.get();
    }
}
