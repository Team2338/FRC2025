package team.gif.robot.subsystems.drivers.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.util.Units;
import team.gif.robot.Constants;

public class TalonSRXTurnMotorEncoder implements TurnMotor, Encoder {
    private TalonSRX motor;

    /**
     * Initiates a TalonSRX with a connected magencoder as
     * and encoder for swerve drive
      * @param id int - the CAN id of the talon
     */
    public TalonSRXTurnMotorEncoder(int id) {
        motor = new TalonSRX(id);
    }

    //Motor configuration
    public void configure(boolean inverted) {
        //---Motor configuration
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(inverted);
    }

    //Encoder configuration
    public void configure() {
        //---Encoder configuration
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        motor.configFeedbackNotContinuous(true, 0);
        motor.configSelectedFeedbackCoefficient(1);
    }

    public double getOutput() {
        return motor.getMotorOutputPercent();
    }

    public void set(double percentOutput) {
        motor.set(ControlMode.PercentOutput, percentOutput);
    }

    public double getTicks() {
        return motor.getSelectedSensorPosition();
    }

    public double getDegrees() {
        return motor.getSelectedSensorPosition() / 4096 * 360 * -1;
    }

    public double getRadians() {
        return motor.getSelectedSensorPosition() / 4096 * 2 * Math.PI * -1;
    }

    public double getVelocity() {
        //ticks per 100ms * 10 (to convert to seconds) / 4096 (to convert to rotations)
        return 0;
        // return motor.getSelectedSensorVelocity() * 10 / 4096;
    }

    public void reset() {
        motor.setSelectedSensorPosition(0);
    }

}
