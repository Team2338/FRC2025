package team.gif.robot.subsystems.drivers.swerve;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import team.gif.robot.Constants;

public class SparkMAXTurnMotor implements TurnMotor{
    private SparkMax motor;

    /**
     * Creates a sparkmax motor (NEO) to be used
     * as a turn motor on a swerve drivetrain.
     * This should never be used except for that purpose.
     * @param id the CAN ID of the motor controller
     */
    public SparkMAXTurnMotor(int id) {
        motor = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);
    }

    public void configure(boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        config.inverted(inverted);
        config.encoder.positionConversionFactor(Constants.ModuleConstants.TURNING_ENCODER_ROT_TO_RAD);
        config.encoder.velocityConversionFactor(Constants.ModuleConstants.TURNING_ENCODER_RPM_2_RAD_PER_SECOND);
        config.smartCurrentLimit(70, 50);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public double getOutput() {
        return motor.getAppliedOutput();

    }

    public void set(double percentOutput) {
        motor.set(percentOutput);

    }
}
