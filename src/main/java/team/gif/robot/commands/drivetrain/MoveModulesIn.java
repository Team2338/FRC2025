package team.gif.robot.commands.drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class MoveModulesIn extends Command {
    public MoveModulesIn() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Move Motors");
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
//        Robot.swerveDrive.modulesTo90();
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        //The % takes care of wheel that are at or above 90 & 270, the -
//        double fLHeading = Math.abs(Robot.swerveDrive.fL.getTurningHeadingDegrees()) % 90;
//        double fRHeading = Math.abs(Robot.swerveDrive.fR.getTurningHeadingDegrees()) % 90;
//        double rLHeading = Math.abs(Robot.swerveDrive.rL.getTurningHeadingDegrees()) % 90;
//        double rRHeading = Math.abs(Robot.swerveDrive.rR.getTurningHeadingDegrees()) % 90;
//        double tolerance = 10;
//        return (fLHeading < tolerance || Math.abs(90 - fLHeading) < tolerance) &&
//                (fRHeading < tolerance || Math.abs(90 - fRHeading) < tolerance) &&
//                (rLHeading < tolerance || Math.abs(90 - rLHeading) < tolerance) &&
//                (rRHeading < tolerance || Math.abs(90 - rRHeading) < tolerance);
        return true;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("End Move");
    }
}
