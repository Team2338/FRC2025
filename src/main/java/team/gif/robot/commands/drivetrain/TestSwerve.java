package team.gif.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class TestSwerve extends Command {

    public TestSwerve() {
        super();
        addRequirements(Robot.swerveDrive);
        //addRequirements(Robot.climber); // uncomment
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
//        SwerveModuleState runningState = new SwerveModuleState(1, new Rotation2d(0));
//        SwerveModuleState stoppedState = new SwerveModuleState(0, new Rotation2d(0));
//
//        SwerveModuleState[] states = {stoppedState, runningState, stoppedState, stoppedState};
//
//        Robot.swerveDrive.setModuleStates(states);

        Robot.swerveDrive.drive(0.3, 0, 0);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.stopModules();
    }
}
