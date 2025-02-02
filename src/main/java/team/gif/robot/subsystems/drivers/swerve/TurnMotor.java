package team.gif.robot.subsystems.drivers.swerve;

public interface TurnMotor {
    //PID, inverted, factory defaults, idle mod, buildin encoder, current limits,

    /**
     * Configures the specified motor controller
     */
    void configure(boolean inverted);

    /**
     * @return double - the percent output of the motor
     */
    double getOutput();

    /**
     * Function to get the output voltage of the motor. Used mainly for SysId.
     * @return double - the current output voltage of the motor
     */
    double getVoltage();

    /**
     *  Sets the speed of the motor controller
     *  @param percentOutput - double - the percent output of the motor
     */
    void set(double percentOutput);

    /**
     * Sets the voltage of the motor controller. This should only be used for the SysId routine
     * @param voltage - double - the voltage to set the motor controller to
     */
    void setVoltage(double voltage);

}
