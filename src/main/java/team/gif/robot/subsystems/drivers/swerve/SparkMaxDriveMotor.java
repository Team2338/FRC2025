package team.gif.robot.subsystems.drivers.swerve;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import team.gif.robot.Constants;

public class SparkMaxDriveMotor implements DriveMotor {
    private SparkMax motor;

    public SparkMaxDriveMotor(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
    }

    public void configure(boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(inverted);
        config.voltageCompensation(12);
        config.encoder.positionConversionFactor(Constants.ModuleConstantsMK3.DRIVE_ENCODER_ROT_2_METER);
        config.encoder.velocityConversionFactor(Constants.ModuleConstantsMK3.DRIVE_ENCODER_RPM_2_METER_PER_SEC);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public double getTemp() {
        return motor.getMotorTemperature();
    }

    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public double getOutput() {
        return motor.getAppliedOutput();
    }

    public void set(double percentOutput) {
        motor.set(percentOutput);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public double getVoltage() {
        //Source: https://www.chiefdelphi.com/t/get-voltage-from-spark-max/344136/5
        return motor.getBusVoltage() * motor.getAppliedOutput();
    }

    public void resetEncoder() {
        motor.getEncoder().setPosition(0);
    }

}
