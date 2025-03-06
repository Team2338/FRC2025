package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class ShooterReverse extends Command {

    public ShooterReverse() {
        super();
        addRequirements(Robot.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.shooter.setShooterBrakeMode();
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        Robot.shooter.runShooterMotor(-Constants.Shooter.REINDEX_PERCENT);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return Robot.shooter.getExitSensorState() && Robot.shooter.getIndexerSensorState();
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.stopShooterMotor();
        Robot.shooter.stopIndexerMotor();
        Robot.shooter.setShooterCoastMode();
    }
}
