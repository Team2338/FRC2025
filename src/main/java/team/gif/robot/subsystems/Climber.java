// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;

public class Climber extends SubsystemBase {
    private TalonFX climberMotor;

    private DoubleSolenoid solenoid;

    private boolean softLimitEnabled;

    public Climber() {
        climberMotor = new TalonFX(RobotMap.CLIMBER_ID);
        configTalon();

        // set up the piston
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMBER_SOLENOID_OUT_PORT, RobotMap.CLIMBER_SOLENOID_IN_PORT);
    }

    public void move(double percentOutput) {
        climberMotor.set(percentOutput);
    }

    public void enableSoftLimit(boolean state) {
        // changing the config of the talon is very expensive, causes loop overruns
        // only update the talon config if it has changed since the last setting
        if (softLimitEnabled != state) {
            SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();

            config.ReverseSoftLimitEnable  = state;
            config.ForwardSoftLimitEnable  = state;
            config.ForwardSoftLimitThreshold = Constants.Climber.FORWARD_SOFT_LIMIT;
            config.ReverseSoftLimitThreshold = Constants.Climber.REVERSE_SOFT_LIMIT;

            climberMotor.getConfigurator().apply(config);
            softLimitEnabled = state;
        }
    }

    public void zeroEncoder() {
        climberMotor.setPosition(0);
    }

    public double getPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }

    public void setPistonOut() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setPistonIn() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Returns the state of the piston as a specialized String for the dashboard
     *
     * @return String value of either "Out" or "In"
     */
    public String getPistonStateAsString() {
        return solenoid.get() == DoubleSolenoid.Value.kForward ? "      Out" : "        In";
    }

    /**
     * Returns the state of the piston as a boolean
     *
     * @return true if Out, false if In
     */
    public Boolean getPistonStateOut() {
        return solenoid.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * Configures the climber Talon motor controller
     */
    private void configTalon() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // invert direction
        config.MotorOutput.Inverted = Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Soft Limits
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Climber.REVERSE_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climber.FORWARD_SOFT_LIMIT;
        softLimitEnabled = true;

        // Write these configs to the TalonFX
        climberMotor.getConfigurator().apply(config);
    }
}