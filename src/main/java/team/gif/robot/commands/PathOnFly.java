package team.gif.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

import java.util.List;

public class PathOnFly extends Command {

    private Pose2d pose;
    private PathConstraints constraints;
    private List waypoints;


    public PathOnFly(boolean isRedAlliance, String reefPosition) {
        super();
        addRequirements(Robot.swerveDrive);
        if (isRedAlliance) {
            switch (reefPosition){
                case "A":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "B":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "C":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "D":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "E":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "F":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "G":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
            }

        } else {
            switch (reefPosition) {
                case "A":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "B":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "C":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "D":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "E":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "F":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
                case "G":
                    pose = new Pose2d(0, 0, new Rotation2d().fromDegrees(0));
                    break;
            }
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pose);

        PathConstraints constraints = new PathConstraints(Constants.ModuleConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND, Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND, Constants.ModuleConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND, Constants.ModuleConstants.MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, Rotation2d.fromDegrees(0))

        );

        path.preventFlipping = true;

        AutoBuilder.followPath(path);

    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {

    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }
}
