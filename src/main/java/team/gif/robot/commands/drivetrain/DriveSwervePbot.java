package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class DriveSwervePbot extends Command {
    private final SlewRateLimiter forwardLimiter;
    private final SlewRateLimiter strafeLimiter;
    private final SlewRateLimiter turnLimiter;

    public DriveSwervePbot() {
        this.forwardLimiter = new SlewRateLimiter(Constants.ModuleConstantsMK3.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.strafeLimiter = new SlewRateLimiter(Constants.ModuleConstantsMK3.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
        this.turnLimiter = new SlewRateLimiter(Constants.ModuleConstantsMK3.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (true) {
            double forwardSign;
            double strafeSign;

            double forward = -Robot.oi.driver.getLeftY(); // need to invert because -Y is away, +Y is pull back
            forward = (Math.abs(forward) > Constants.Joystick.DEADBAND) ? forward : 0.0; //0.00001;

            double strafe = -Robot.oi.driver.getLeftX(); // need to invert because -X is left, +X is right
            strafe = (Math.abs(strafe) > Constants.Joystick.DEADBAND) ? strafe : 0.0;

            double rot = -Robot.oi.driver.getRightX(); // need to invert because left is negative, right is positive
            rot = (Math.abs(rot) > Constants.Joystick.DEADBAND) ? rot : 0.0;

            forwardSign = forward/Math.abs(forward);
            strafeSign = strafe/Math.abs(strafe);
            // Use a parabolic curve (instead if linear) for the joystick to speed ratio
            // This allows for small joystick inputs to use slower speeds
            forward = Math.abs(forward) * forward;
            strafe = Math.abs(strafe) * strafe;

            forward = .5 * Math.sqrt(2 + forward*forward - strafe*strafe + 2*forward*Math.sqrt(2)) -
                    .5 * Math.sqrt(2 + forward*forward - strafe*strafe - 2*forward*Math.sqrt(2));

            strafe = .5 * Math.sqrt(2 - forward*forward + strafe*strafe + 2*strafe*Math.sqrt(2)) -
                    .5 * Math.sqrt(2 - forward*forward + strafe*strafe - 2*strafe*Math.sqrt(2));

            if( Double.isNaN(forward) )
                forward = forwardSign;
            if( Double.isNaN(strafe) )
                strafe = strafeSign;

            //Forward speed, Sideways speed, Rotation Speed
            forward = forwardLimiter.calculate(forward) * Constants.ModuleConstantsMK3.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
            strafe = strafeLimiter.calculate(strafe) * Constants.ModuleConstantsMK3.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND;
            rot = turnLimiter.calculate(rot) * Constants.ModuleConstantsMK3.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

            // the robot starts facing the driver station so for this year negating y and x
            Robot.swerveDrive.drive(forward, strafe, rot);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
