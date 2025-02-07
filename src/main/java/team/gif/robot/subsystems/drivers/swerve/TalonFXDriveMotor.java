package team.gif.robot.subsystems.drivers.swerve;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import team.gif.robot.Constants;

public class TalonFXDriveMotor implements DriveMotor{
    private TalonFX motor;

    public TalonFXDriveMotor(int motorID) {
        motor = new TalonFX(motorID);
    }

    /**
     * Configures a TalonFX to be a drive motor on a swerve drivetrain.
     * Sets empty TalonFX config, and sets it to brake mode.
     */
    public void configure(boolean inverted) {
        TalonFXConfigurator talonFXConfig = motor.getConfigurator();
        FeedbackConfigs config = new TalonFXConfiguration().Feedback.withSensorToMechanismRatio(Constants.ModuleConstants.GEAR_RATIO);

        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        talonFXConfig.apply(motorConfigs);
        talonFXConfig.apply(config);
    }

    public double getTemp() {
        return motor.getDeviceTemp().getValueAsDouble();
    }


    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble() * Constants.ModuleConstants.DRIVE_ENCODER_ROT_2_METER;
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

    public void resetEncoder() {
        motor.setPosition(0);
    }

}
