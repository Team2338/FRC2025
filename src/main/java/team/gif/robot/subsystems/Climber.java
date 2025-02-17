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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Climber extends SubsystemBase {
    private TalonFX climber;
    private TalonFXConfigurator talonFXConfig;
    private SoftwareLimitSwitchConfigs softLimitConfig;
    private DoubleSolenoid solenoid;

    public Climber() {
        climber = new TalonFX(RobotMap.CLIMBER_ID);


        talonFXConfig = climber.getConfigurator();
        softLimitConfig = new TalonFXConfiguration().SoftwareLimitSwitch;

        softLimitConfig.withForwardSoftLimitThreshold(Constants.Climber.FORWARD_SOFT_LIMIT);
        softLimitConfig.withReverseSoftLimitThreshold(Constants.Climber.REVERSE_SOFT_LIMIT);

        softLimitConfig.withForwardSoftLimitEnable(true);
        softLimitConfig.withReverseSoftLimitEnable(true);

        MotorOutputConfigs motorConfigs = new TalonFXConfiguration().MotorOutput;
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        talonFXConfig.apply(motorConfigs);
        talonFXConfig.apply(softLimitConfig);

        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMBER_SOLENOID_IN_PORT, RobotMap.CLIMBER_SOLENOID_OUT_PORT);
    }

    public void runClimber(double percentOutput) {
        climber.set(percentOutput);
    }

    public void enableSoftLimit(boolean enable) {
        softLimitConfig.withForwardSoftLimitEnable(enable);
        softLimitConfig.withReverseSoftLimitEnable(enable);
        talonFXConfig.apply(softLimitConfig);
    }

    public void resetEncoder() {
        climber.setPosition(0);
    }

    public double getPosition() {
        return climber.getPosition().getValueAsDouble();
    }

    public void setPistonOut() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

}