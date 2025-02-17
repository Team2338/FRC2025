package team.gif.robot.subsystems.drivers.swerve;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFXTurnMotor implements TurnMotor {
    private TalonFX motor;

    public TalonFXTurnMotor(int motorID) {
        motor = new TalonFX(motorID);
    }

    public void configure(boolean inverted) {
        TalonFXConfigurator talonFXConfig = motor.getConfigurator();

        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        talonFXConfig.apply(motorConfigs);
    }

    public double getOutput() {
        return motor.get();
    }

    public void set(double percentOutput) {
        motor.set(percentOutput);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }
}
