// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private static TalonSRX shooter;
    private static TalonSRX indexer;
    private double shooterSpeed;
    private double indexerSpeed;


    /** Creates a new ExampleSubsystem. */
    public Shooter() {
        shooter = new TalonSRX(RobotMap.SHOOTER_ID);
        shooter.configFactoryDefault();
        shooter.setNeutralMode(NeutralMode.Coast);
        shooter.setInverted(true);

        indexer= new TalonSRX(RobotMap.INDEXER_ID);
        indexer.configFactoryDefault();
        indexer.setNeutralMode(NeutralMode.Coast);

        SmartDashboard.putNumber("Shooter Speed", 0);
        SmartDashboard.putNumber("Indexer Speed", 0);
        shooterSpeed = SmartDashboard.getNumber("Shooter Speed", 0);
        indexerSpeed = SmartDashboard.getNumber("Indexer Speed", 0);
    }

    public void moveMotor(double percentOutput) {

        shooter.set(TalonSRXControlMode.PercentOutput, percentOutput);
    }

    public void moveIndexerFromShuffleboard() {
        indexer.set(TalonSRXControlMode.PercentOutput, indexerSpeed);
    }

    public void moveFromShuffleboard() {
        shooter.set(TalonSRXControlMode.PercentOutput, shooterSpeed);
    }

}
