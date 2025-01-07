package team.gif.robot.subsystems.drivers.swerve;

public interface DriveMotor {
    /**
     * Complete configuration of the drive motor,
     * including neutral/idle mode, pid, limits, etc
     */
    void configure(boolean inverted);

    /**
     * Gets the temperature of the motor, to make sure we don't overheat
     * @return ºC as a double
     */
    double getTemp();

    /**
     * Gets the current velocity of the wheel (not motor)
     * @return The current velocity of the wheel in RPM
     */
    double getVelocity();

    /**
     * The accumulative distance the module has traveled in meters.
     * @return double - the position of the encoder in meters
     */
    double getPosition();

    /**
     * Function to get the current output of the motor
     * @return Double between -1 and 1
     */
    double getOutput();

    /**
     * Sets the speed of the motor to a specified
     * percent value
     * @param percentOutput - value -1 to 1
     */
    void set(double percentOutput);

    /**
     * Resets the encoder position to 0
     */
    void resetEncoder();

}
