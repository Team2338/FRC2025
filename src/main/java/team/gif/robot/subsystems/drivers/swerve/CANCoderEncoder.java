package team.gif.robot.subsystems.drivers.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;


public class CANCoderEncoder implements Encoder{
    private CANcoder encoder;

    /**
     * Creates a CANCoder for use as a
     * turn encoder in a swerve drivetrain
     * @param id - The CAN ID of the encoder
     */
    public CANCoderEncoder(int id) {
        encoder = new CANcoder(id);
    }

    public void configure() {
        MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5);
        encoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(encoderConfig));

    }

    /**
     * Returns the position of the CANcoder as a value from -0.5-0.5
     * @return double - current encoder position
     */
    public double getTicks() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Returns the current position in degrees rom -180-180
     * @return double - position of the wheel in degrees
     */
    public double getDegrees() {
        return getTicks() * 360;

    }

    public double getRadians() {
        return getTicks() * 2 * Math.PI;
    }

    public double getVelocity() {
        return encoder.getVelocity().getValueAsDouble();
    }

    public void reset() {
        encoder.setPosition(0);
    }

}
