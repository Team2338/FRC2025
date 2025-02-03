package team.gif.robot.subsystems.drivers.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import team.gif.robot.Constants;
import team.gif.robot.subsystems.SwerveDrivetrainMk3;

public class SwerveModule {
    /* ------ Devices ------*/
    private final DriveMotor driveMotor;
    private final TurnMotor turnMotor;
    private final Encoder encoder;

    /* ----- PID Constants -----*/
    private final double P;
    private final SimpleMotorFeedforward driveFF;
    private final double turnFF;

    private double turnOffset;

    public SwerveModule(
            DriveMotor driveMotor,
            TurnMotor turnMotor,
            Encoder encoder,
            boolean isTurnInverted,
            boolean isDriveInverted,
            double turningOffset,
            SimpleMotorFeedforward driveFF,
            double turnFF,
            double P
    ) {

        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.encoder = encoder;

        this.driveMotor.configure(isDriveInverted);
        this.turnMotor.configure(isTurnInverted);
        this.encoder.configure();

        this.turnOffset = turningOffset;
        this.P = P;

        this.driveFF = driveFF;
        this.turnFF = turnFF;
    }

    /**
     * Get the active state of the swerve module
     * @return Returns the active state of the given swerveModule
     */
    public SwerveModuleState getState() {
        //We divide velocity by 2π because it is returned
        //in revolutions per second and we need it
        //in radians per seconds
        return new SwerveModuleState(driveMotor.getVelocity(),
                new Rotation2d(getTurningHeading()));
    }

    public double getRawHeading() {
        return encoder.getTicks();
    }

    public double encoderDegrees() {
        return encoder.getDegrees();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity();
    }
    /**
     * Get the heading of the swerve module
     * @return Returns the heading of the module in radians as a double
     */
    public double getTurningHeading() {
        //TODO: Redo the offset so that we can get rid of the radian conversion
        //TODO: I got rid of the is Abs inverted, do we need this??
        double heading = Units.degreesToRadians(getTurningHeadingDegrees());
        heading %= 2 * Math.PI;
        return heading;
    }

    /**
     * Get the heading of the swerve module
     * @return Returns the heading of the module in degrees as a double
     */
    public double getTurningHeadingDegrees() {
        return encoder.getDegrees() - turnOffset;
    }

    /**
     * Reset the wheel to its 0 positions
     */
    public void resetWheel() {
        final double error = getTurningHeading();
        final double ff = turnFF * Math.abs(error) / error;
        final double turnOutput = ff + (P * error);

        turnMotor.set(turnOutput);
    }

    /**
     * Find the reverse of a given angle (i.e. pi/4->7pi/4)
     * @param radians the angle in radians to reverse
     * @return the reversed angle
     */
    private double findRevAngle(double radians) {
        return (Math.PI * 2 + radians) % (2 * Math.PI) - Math.PI;
    }

    /**
     * Finds the distance in ticks between two setpoints
     * @param setpoint initial/current point
     * @param position desired position
     * @return the distance between the two point
     */
    private double getDistance(double setpoint, double position) {
        return Math.abs(setpoint - position);
    }

    /**
     * Optimize the swerve module state by setting it to the closest equivalent vector
     * @param original the original swerve module state
     * @return the optimized swerve module state
     */
    private SwerveModuleState optimizeState(SwerveModuleState original) {
        //This code is basically magic. I aint touching it

        // Compute all options for a setpoint
        double position = getTurningHeading();
        double setpoint = original.angle.getRadians();
        double forward = setpoint + (2 * Math.PI);
        double reverse = setpoint - (2 * Math.PI);
        double antisetpoint = findRevAngle(setpoint);
        double antiforward = antisetpoint + (2 * Math.PI);
        double antireverse = antisetpoint - (2 * Math.PI);

        // Find setpoint option with minimum distance
        double[] alternatives = { forward, reverse, antisetpoint, antiforward, antireverse };
        double min = setpoint;
        double minDistance = getDistance(setpoint, position);
        int minIndex = -1;
        for (int i = 0; i < alternatives.length; i++) {
            double dist = getDistance(alternatives[i], position);
            if (dist < minDistance) {
                min = alternatives[i];
                minDistance = dist;
                minIndex = i;
            }
        }

        // Figure out the speed. Anti- directions should be negative.
        double speed = original.speedMetersPerSecond;
        if (minIndex > 1) {
            speed *= -1;
        }

        return new SwerveModuleState(speed, new Rotation2d(min));
    }

    /**
     * Set the desired state of the swerve module
     * @param state The desired state of the swerve module
     */
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState stateOptimized = optimizeState(state);
        double driveOutput = driveFF.calculate(stateOptimized.speedMetersPerSecond);
        final double error = getTurningHeading() - stateOptimized.angle.getRadians();

        //if error is negative, FF should also be negative
        final double ff = turnFF * Math.abs(error) / error;
        //accum += error;
        final double turnOutput = ff + (P * error);
        driveMotor.setVoltage(driveOutput);
        turnMotor.set(turnOutput);
    }

    /**
     * Stop the swerve modules
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**
     * Get the position of the swerve module - TODO: HAS BUG
     * @return the position of the swerve module
     *
     * Process:
     * Set adjust to 1.0
     * Create auto with the longest straight line possible
     * adjust = actual distance / desired distance
     *
     */
    public SwerveModulePosition getPosition() {

        //TODO: idk why we multiply by 2176.5
        double adjust = 0.9578661376;
        return new SwerveModulePosition(driveMotor.getPosition(), new Rotation2d(getTurningHeading()));
    }

    /**
     * Resets the drive encoder
     */
    public void resetDriveEncoders() {
        driveMotor.resetEncoder();
    }

    public boolean isDriveMotorCool() {
        if (driveMotor.getTemp() >= Constants.MotorTemps.DRIVETRAIN_MOTOR_TEMP) {
            return false;
        }
        return true;
    }


}
