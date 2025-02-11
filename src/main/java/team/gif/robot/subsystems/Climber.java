// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Climber extends SubsystemBase {
    private TalonFX climber;
    /** Creates a new ExampleSubsystem. */
    public Climber() {
        climber = new TalonFX(RobotMap.CLIMBER_ID);

        TalonFXConfigurator talonFXConfig = climber.getConfigurator();
        SoftwareLimitSwitchConfigs softLimitConfig = new TalonFXConfiguration().SoftwareLimitSwitch;

        softLimitConfig.withForwardSoftLimitThreshold(6);
        softLimitConfig.withReverseSoftLimitThreshold(0);

        softLimitConfig.withForwardSoftLimitEnable(true);
        softLimitConfig.withReverseSoftLimitEnable(true);


        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        talonFXConfig.apply(motorConfigs);
        talonFXConfig.apply(softLimitConfig);

    }

    public void runClimber(double percentOutput) {
        climber.set(percentOutput);

    }






}
