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
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Climber extends SubsystemBase {
    private TalonFX climber;
    private TalonFXConfigurator talonFXConfig;
    private SoftwareLimitSwitchConfigs softLimitConfig;
    private SysIdRoutine sysIdRoutine;

    public Climber() {
        climber = new TalonFX(RobotMap.CLIMBER_ID);

        TalonFXConfigurator talonFXConfig = climber.getConfigurator();
        SoftwareLimitSwitchConfigs softLimitConfig = new TalonFXConfiguration().SoftwareLimitSwitch;

        softLimitConfig.withForwardSoftLimitThreshold(Constants.Climber.FORWARD_SOFT_LIMIT);
        softLimitConfig.withReverseSoftLimitThreshold(Constants.Climber.REVERSE_SOFT_LIMIT);

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

    public void runClimberVoltage(double voltage){
        climber.setVoltage(voltage);
    }

    public void disableSoftLimit() {
        softLimitConfig.withForwardSoftLimitEnable(false);
        softLimitConfig.withReverseSoftLimitEnable(false);
        talonFXConfig.apply(softLimitConfig);
    }

    public void resetEncoder() {
        climber.setPosition(0);
    }

    public double getPosition() {
        return climber.getPosition().getValueAsDouble();
    }

    private SysIdRoutine getSysIdRoutine() {
        MutVoltage voltMut = Volts.mutable(0);
        MutAngle angleMut = Radians.mutable(0);
        MutAngularVelocity vMut= RadiansPerSecond.mutable(0);

        return new SysIdRoutine(
                new SysIdRoutine.Config(),

                new SysIdRoutine.Mechanism(
                        voltage -> {
                            climber.setVoltage(voltage.baseUnitMagnitude());
                        },
                        log -> {
                            log.motor("Climber")
                                    .voltage(voltMut.mut_replace(climber.getMotorVoltage().getValueAsDouble(), Volts))
                                    .angularPosition(angleMut.mut_replace(climber.getPosition().getValueAsDouble(),Radians))
                                    .angularVelocity(vMut.mut_replace(climber.getVelocity().getValueAsDouble(), RadiansPerSecond ));
                        },
                        this));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return getSysIdRoutine().quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return getSysIdRoutine().dynamic(direction);
    }
}