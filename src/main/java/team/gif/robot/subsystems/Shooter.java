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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.LaserCANSensor;
import team.gif.robot.subsystems.drivers.ToFSensor;

public class Shooter extends SubsystemBase {
    private static TalonSRX shooter;
    private static TalonSRX indexer;
    private ShuffleboardTab tab = Shuffleboard.getTab("FRC 2025");
    private static LaserCANSensor sensorTop;
    private static ToFSensor sensorBottom;
    private double shooterSpeed =
            tab.add("Shooter Speed", .5).withWidget(BuiltInWidgets.kNumberSlider)
                    .getEntry().getDouble(0);
    private double indexerSpeed =
            tab.add("Indexer Speed", .22).withWidget(BuiltInWidgets.kNumberSlider)
                    .getEntry().getDouble(0);


    /** Creates a new ExampleSubsystem. */
    public Shooter() {
        shooter = new TalonSRX(RobotMap.SHOOTER_ID);
        sensorTop = new LaserCANSensor(RobotMap.SENSOR_TOP_ID);
        sensorBottom = new ToFSensor(RobotMap.SENSOR_BOTTOM_ID);
        shooter.configFactoryDefault();
        shooter.setNeutralMode(NeutralMode.Coast);
        shooter.setInverted(true);

        indexer= new TalonSRX(RobotMap.INDEXER_ID);
        indexer.configFactoryDefault();
        indexer.setNeutralMode(NeutralMode.Coast);
    }

    public void moveMotor(double percentOutput) {
        shooter.set(TalonSRXControlMode.PercentOutput, percentOutput);
    }

    public boolean isFireReady() {
        return (sensorLeftActive() && sensorRightActive());
    }

    public boolean sensorLeftActive() {
        return sensorTop.getDistance() < Constants.Shooter.TARGET_DISTANCE_MM;
    }

    public boolean sensorRightActive() {
        return sensorBottom.getDistance() < Constants.Shooter.TARGET_DISTANCE_MM;
    }

    public void moveIndexerFromShuffleboard() {
        indexer.set(TalonSRXControlMode.PercentOutput, indexerSpeed);
    }

    public void moveFromShuffleboard() {
        shooter.set(TalonSRXControlMode.PercentOutput, shooterSpeed);
    }
}
