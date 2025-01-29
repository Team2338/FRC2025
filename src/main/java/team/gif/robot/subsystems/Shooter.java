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
    private DigitalInput firstSensor;
    private DigitalInput secondSensor;


    /** Creates a new ExampleSubsystem. */
    public Shooter() {
        shooter = new TalonSRX(RobotMap.SHOOTER_ID);
        shooter.configFactoryDefault();
        shooter.setNeutralMode(NeutralMode.Coast);
        shooter.setInverted(true);

        indexer= new TalonSRX(RobotMap.INDEXER_ID);
        indexer.configFactoryDefault();
        indexer.setNeutralMode(NeutralMode.Coast);

        firstSensor = new DigitalInput(RobotMap.SHOOTER_SENSOR_FIRST_ID);
        secondSensor = new DigitalInput(RobotMap.SHOOTER_SENSOR_SECOND_ID);

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

    public boolean getFirstSensorState() {
        return firstSensor.get();
    }

    public boolean getSecondSensorState() {
        return secondSensor.get();
    }
}
