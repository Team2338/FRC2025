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
     *  Sets the speed of the motor controller
     */
    void set(double percentOutput);

}
