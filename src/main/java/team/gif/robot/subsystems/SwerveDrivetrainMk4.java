package team.gif.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.lib.LimelightHelpers;
import team.gif.lib.drivePace;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;
import team.gif.robot.subsystems.drivers.swerve.CANCoderEncoder;
import team.gif.robot.subsystems.drivers.swerve.DriveMotor;
import team.gif.robot.subsystems.drivers.swerve.Encoder;
import team.gif.robot.subsystems.drivers.swerve.SwerveModule;
import team.gif.robot.subsystems.drivers.swerve.TalonFXDriveMotor;
import team.gif.robot.subsystems.drivers.swerve.TalonFXTurnMotor;
import team.gif.robot.subsystems.drivers.swerve.TurnMotor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

/**
 * @author Rohan Cherukuri
 * @since 2/14/22
 */
public class SwerveDrivetrainMk4 extends SubsystemBase {
    public static SwerveModule fL;
    public static SwerveModule fR;
    public static SwerveModule rL;
    public static SwerveModule rR;

    private static DriveMotor fLDriveMotor;
    private static DriveMotor fRDriveMotor;
    private static DriveMotor rLDriveMotor;
    private static DriveMotor rRDriveMotor;

    private static TurnMotor fLTurnMotor;
    private static TurnMotor fRTurnMotor;
    private static TurnMotor rLTurnMotor;
    private static TurnMotor rRTurnMotor;

    private static Encoder fLEncoder;
    private static Encoder fREncoder;
    private static Encoder rLEncoder;
    private static Encoder rREncoder;

    public SwerveDrivePoseEstimator poseEstimator;
    private drivePace drivePace;

    // Network Table publishers for the swerve
    // states so that we can use them in advantage scope
    private static final StructArrayPublisher<SwerveModuleState> targetPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("TargetSwerveState", SwerveModuleState.struct).publish();
    private static final StructArrayPublisher<SwerveModuleState> actualPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("ActualSwerveState", SwerveModuleState.struct).publish();
    private static final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("EstimatedPose", Pose2d.struct).publish();
    private static final StructPublisher<ChassisSpeeds> chassisSpeedsStructPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();


    /**
     * Constructor for swerve drivetrain using 4 swerve modules using Kraken x60s to drive and Falcon500 to turn
     */
    public SwerveDrivetrainMk4() {
        super();

        configModules();

        resetDriveEncoders();

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Drivetrain.DRIVE_KINEMATICS, Robot.pigeon.getRotation2d(), getPosition(), new Pose2d(0, 0, new Rotation2d(0)));

        drivePace = drivePace.COAST_FR;

        configPathPlanner();
    }

    /**
     * Periodic function
     * - constantly update the odometry
     */
    @Override
    public void periodic() {
        poseEstimator.update(
            Robot.pigeon.getRotation2d(),
            getPosition()
        );


        LimelightHelpers.PoseEstimate collectEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-collect");
        LimelightHelpers.PoseEstimate shooterEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
        boolean ignoreCollectEstimate = true;
        boolean ignoreShooterEstimate = true;
        double yawRate = Robot.pigeon.getYawRate();

        if(collectEstimate != null && collectEstimate.tagCount > 0 && yawRate < 720) {
            ignoreCollectEstimate = false;
        }
        if(shooterEstimate != null && shooterEstimate.tagCount > 0 && yawRate < 720) {
            ignoreShooterEstimate = false;
        }
        if(!ignoreCollectEstimate) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            poseEstimator.addVisionMeasurement(
                    collectEstimate.pose,
                    collectEstimate.timestampSeconds);
        }
        if(!ignoreShooterEstimate) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            poseEstimator.addVisionMeasurement(
                    shooterEstimate.pose,
                    shooterEstimate.timestampSeconds);
        }



        if (Robot.fullDashboard) {
            posePublisher.set(poseEstimator.getEstimatedPosition());
            updateShuffleboardDebug("Swerve");
        }
    }

    /**
     * Reset the odometry to a given pose
     *
     * @param pose the pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(Robot.pigeon.getRotation2d(), new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()}, pose);
    }

    /**
     * Get the robot relative speed
     * @return ChassisSpeeds of the robot relative speed
     */
    public ChassisSpeeds getRobotRelativeSpeed() {
        SwerveModuleState frontLeftState = new SwerveModuleState(fL.getDriveVelocity(), Rotation2d.fromDegrees(fL.getTurningHeadingDegrees()));
        SwerveModuleState frontRightState = new SwerveModuleState(fR.getDriveVelocity(), Rotation2d.fromDegrees(fR.getTurningHeadingDegrees()));
        SwerveModuleState rearLeft = new SwerveModuleState(rL.getDriveVelocity(), Rotation2d.fromDegrees(rL.getTurningHeadingDegrees()));
        SwerveModuleState rearRight = new SwerveModuleState(rR.getDriveVelocity(), Rotation2d.fromDegrees(rR.getTurningHeadingDegrees()));

        ChassisSpeeds speed = Constants.Drivetrain.DRIVE_KINEMATICS.toChassisSpeeds(frontLeftState, frontRightState, rearLeft, rearRight);

        if (Robot.fullDashboard) {
            chassisSpeedsStructPublisher.set(speed);
        }

        return speed;
    }

    /**
     * Drive the bot with given params - always field relative
     *
     * @param x   dForward
     * @param y   dLeft
     * @param rot dRot
     */
    public void drive(double x, double y, double rot) {
        SwerveModuleState[] swerveModuleStates =
                Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(
                        drivePace.getIsFieldRelative() ?
                                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, Robot.pigeon.getRotation2d())
                                : new ChassisSpeeds(x, y, rot));
        if (Robot.fullDashboard) {
            SwerveModuleState[] actualStates = { fL.getState(), fR.getState(), rL.getState(), rR.getState()};
            targetPublisher.set(swerveModuleStates);
            actualPublisher.set(actualStates);
        }
        setModuleStates(swerveModuleStates);
    }

    public void setMaxDrive() {
        fLDriveMotor.set(1);
        fRDriveMotor.set(1);
        rLDriveMotor.set(1);
        rRDriveMotor.set(1);
    }

    public void stopDrive() {
        drive(0,0,0);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a SwerveModuleState array
     *
     * @param desiredStates SwerveModuleState array of desired states for each of the modules
     * @implNote Only for use in the SwerveDrivetrain class and the RobotTrajectory Singleton, for any general use {@link SwerveDrivetrainMk4#drive(double, double, double)}
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, drivePace.getValue()
        );

        fL.setDesiredState(desiredStates[0]);
        fR.setDesiredState(desiredStates[1]);
        rL.setDesiredState(desiredStates[2]);
        rR.setDesiredState(desiredStates[3]);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a ChassisSpeeds class
     * @param chassisSpeeds Robot Relative ChassisSpeeds to apply to wheel speeds
     * @implNote Use only in {@link SwerveDrivetrainMk4}
     */
    public void setModuleChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Drivetrain.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, drivePace.getValue()
        );

        for (SwerveModuleState state : swerveModuleStates) {
            state.speedMetersPerSecond = Math.min(state.speedMetersPerSecond, drivePace.getValue());
        }

        fL.setDesiredState(swerveModuleStates[0]);
        fR.setDesiredState(swerveModuleStates[1]);
        rL.setDesiredState(swerveModuleStates[2]);
        rR.setDesiredState(swerveModuleStates[3]);

        if(Robot.fullDashboard) {
            chassisSpeedsStructPublisher.set(chassisSpeeds);
            targetPublisher.set(swerveModuleStates);
        }
    }

    /**
     * This set moves all the modules to 90 degrees. It turns the modules inward to prevent the robot from moving
     */
    public void modulesTo90() {
        SwerveModuleState state90 = new SwerveModuleState(0, Rotation2d.fromDegrees(90));
        fL.setDesiredState(state90, true);
        fR.setDesiredState(state90, false);
        rL.setDesiredState(state90, true);
        rR.setDesiredState(state90, false);

    }

    /**
     * Get the current pose of the robot
     *
     * @return The current pose of the robot (Pose2D)
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Stop all the modules
     */
    public void stopModules() {
        fL.stop();
        fR.stop();
        rR.stop();
        rL.stop();
    }

    /**
     * Get the current position of each of the swerve modules
     *
     * @return An array in form fL -> fR -> rL -> rR of each of the module positions
     */
    public SwerveModulePosition[] getPosition() {

        return new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()};
    }

    /**
     * Reset the drive encoders
     */
    public void resetDriveEncoders() {
        fL.resetDriveEncoders();
        fR.resetDriveEncoders();
        rL.resetDriveEncoders();
        rR.resetDriveEncoders();
    }

    /**
     * set the drivePace settings for the drivebase
     *
     * @param drivePace the drivePace to set
     */
    public void setDrivePace(drivePace drivePace) {
        this.drivePace = drivePace;
    }

    /**
     * Get the current drivePace settings
     *
     * @return the current drivePace settings
     */
    public drivePace getDrivePace() {
        return drivePace;
    }

    public double getPoseX() {
        return getPose().getX();
    }

    public double getPoseY() {
        return getPose().getY();
    }

    private static void configModules() {
        fLDriveMotor = new TalonFXDriveMotor(RobotMap.FRONT_LEFT_DRIVE_MOTOR_ID);
        fLTurnMotor = new TalonFXTurnMotor(RobotMap.FRONT_LEFT_TURNING_MOTOR_ID);
        fLEncoder = new CANCoderEncoder(RobotMap.FRONT_LEFT_CANCODER_ID);
        fRDriveMotor = new TalonFXDriveMotor(RobotMap.FRONT_RIGHT_DRIVE_MOTOR_ID);
        fRTurnMotor = new TalonFXTurnMotor(RobotMap.FRONT_RIGHT_TURNING_MOTOR_ID);
        fREncoder = new CANCoderEncoder(RobotMap.FRONT_RIGHT_CANCODER_ID);
        rLDriveMotor = new TalonFXDriveMotor(RobotMap.REAR_LEFT_DRIVE_MOTOR_ID);
        rLTurnMotor = new TalonFXTurnMotor(RobotMap.REAR_LEFT_TURNING_MOTOR_ID);
        rLEncoder = new CANCoderEncoder(RobotMap.REAR_LEFT_CANCODER_ID);
        rRDriveMotor = new TalonFXDriveMotor(RobotMap.REAR_RIGHT_DRIVE_MOTOR_ID);
        rRTurnMotor = new TalonFXTurnMotor(RobotMap.REAR_RIGHT_TURNING_MOTOR_ID);
        rREncoder = new CANCoderEncoder(RobotMap.REAR_RIGHT_CANCODER_ID);


        fL = new SwerveModule(
                fLDriveMotor,
                fLTurnMotor,
                fLEncoder,
                true,
                true,
                Constants.Drivetrain.FRONT_LEFT_OFFSET,
                Constants.ModuleConstants.DrivetrainPID.fLDriveFF,
                Constants.ModuleConstants.DrivetrainPID.frontLeftFF,
                Constants.ModuleConstants.DrivetrainPID.frontLeftP
        );

        fR = new SwerveModule (
                fRDriveMotor,
                fRTurnMotor,
                fREncoder,
                true,
                true,
                Constants.Drivetrain.FRONT_RIGHT_OFFSET,
                Constants.ModuleConstants.DrivetrainPID.fRDriveFF,
                Constants.ModuleConstants.DrivetrainPID.frontRightFF,
                Constants.ModuleConstants.DrivetrainPID.frontRightP
        );

        rL = new SwerveModule (
                rLDriveMotor,
                rLTurnMotor,
                rLEncoder,
                true,
                true,
                Constants.Drivetrain.REAR_LEFT_OFFSET,
                Constants.ModuleConstants.DrivetrainPID.rLDriveFF,
                Constants.ModuleConstants.DrivetrainPID.rearLeftFF,
                Constants.ModuleConstants.DrivetrainPID.rearLeftP
        );

        rR = new SwerveModule (
                rRDriveMotor,
                rRTurnMotor,
                rREncoder,
                true,
                true,
                Constants.Drivetrain.REAR_RIGHT_OFFSET,
                Constants.ModuleConstants.DrivetrainPID.rRDriveFF,
                Constants.ModuleConstants.DrivetrainPID.rearRightFF,
                Constants.ModuleConstants.DrivetrainPID.rearRightP
        );
    }

    private void configPathPlanner() {
        RobotConfig ppConfig;
        try{
            ppConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            ModuleConfig moduleConfig = new ModuleConfig(
                    Constants.ModuleConstants.WHEEL_DIAMETER_METERS / 2,
                    Constants.ModuleConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND,
                    Constants.ModuleConstants.COEFFICIENT_OF_FRICTION,
                    DCMotor.getKrakenX60(1),
                    50,
                    1);
            ppConfig = new RobotConfig(68, 6.883, moduleConfig, Constants.Drivetrain.TRACK_WIDTH);
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeed,
                this::setModuleChassisSpeeds,
                Constants.DrivetrainAuto.AUTO_DRIVE_CONTROLLER,
                ppConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if( alliance.isPresent() ){
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
    }

    public double fLDriveTemp() { return fLDriveMotor.getTemp(); }
    public double fRDriveTemp() { return fRDriveMotor.getTemp(); }
    public double rLDriveTemp() { return rLDriveMotor.getTemp(); }
    public double rRDriveTemp() { return rRDriveMotor.getTemp(); }

    public void updateShuffleboardDebug(String shuffleboardTabName) {

        SmartDashboard.putData(shuffleboardTabName + "/FL Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", fL::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData(shuffleboardTabName + "/FR Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", fR::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData(shuffleboardTabName + "/RL Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", rL::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData(shuffleboardTabName + "/RR Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", rR::getTurningHeadingDegrees, null);
        });


        SmartDashboard.putNumber(shuffleboardTabName + "/FR Raw Degrees", fR.encoderDegrees());
        SmartDashboard.putNumber(shuffleboardTabName + "/FL Raw Degrees", fL.encoderDegrees());
        SmartDashboard.putNumber(shuffleboardTabName + "/RR Raw Degrees", rR.encoderDegrees());
        SmartDashboard.putNumber(shuffleboardTabName + "/RL Raw Degrees", rL.encoderDegrees());

        SmartDashboard.putNumber(shuffleboardTabName + "/FL Raw Encoder", fLEncoder.getTicks());
        SmartDashboard.putNumber(shuffleboardTabName + "/FR Raw Encoder", fREncoder.getTicks());
        SmartDashboard.putNumber(shuffleboardTabName + "/RL Raw Encoder", rLEncoder.getTicks());
        SmartDashboard.putNumber(shuffleboardTabName + "/RR Raw Encoder", rREncoder.getTicks());

        SmartDashboard.putNumber(shuffleboardTabName + "/FR Raw Radians", fR.getTurningHeading());
        SmartDashboard.putNumber(shuffleboardTabName + "/FL Raw Radians", fL.getTurningHeading());
        SmartDashboard.putNumber(shuffleboardTabName + "/RR Raw Radians", rR.getTurningHeading());
        SmartDashboard.putNumber(shuffleboardTabName + "/RL Raw Radians", rL.getTurningHeading());

        SmartDashboard.putNumber(shuffleboardTabName + "/FL Drive Encoder", fLDriveMotor.getPosition());
        SmartDashboard.putNumber(shuffleboardTabName + "/FR Drive Encoder", fRDriveMotor.getPosition());
        SmartDashboard.putNumber(shuffleboardTabName + "/RL Drive Encoder", rLDriveMotor.getPosition());
        SmartDashboard.putNumber(shuffleboardTabName + "/RR Drive Encoder", rRDriveMotor.getPosition());

        SmartDashboard.putNumber(shuffleboardTabName + "/FL Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber(shuffleboardTabName + "/FR Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber(shuffleboardTabName + "/RL Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber(shuffleboardTabName + "/RR Rot Output", fLTurnMotor.getOutput());


        //TODO: Add target to shuffleboard
    }

    public SysIdRoutine getSysIdRoutine(String motors) {
        MutVoltage voltMut = Volts.mutable(0);


        if (motors.equals("drive")) {
            MutDistance posMut = Meters.mutable(0);
            MutLinearVelocity vMut= MetersPerSecond.mutable(0);

            return new SysIdRoutine(new SysIdRoutine.Config(null, voltMut.mut_replace(5, Volts), null),
                    new SysIdRoutine.Mechanism(
                            voltage -> {
                                fLDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                fRDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                rLDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                rRDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                            },
                            log -> {
                                log.motor("fLDrive")
                                        .voltage(voltMut.mut_replace(fLDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(fLDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(fLDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("fRDrive")
                                        .voltage(voltMut.mut_replace(fRDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(fRDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(fRDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("rLDrive")
                                        .voltage(voltMut.mut_replace(rLDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(rLDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(rLDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("rRDrive")
                                        .voltage(voltMut.mut_replace(rRDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(rRDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(rRDriveMotor.getVelocity(), MetersPerSecond));

                            },
                            this));
        } else if (motors.equals("turn")) {
            MutAngle thetaMut = Radians.mutable(0);
            MutAngularVelocity thetaVMut = RadiansPerSecond.mutable(0);

            return new SysIdRoutine(new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                    voltage -> {
                            fLTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            fRTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            rLTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            rRTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                        }, log -> {
                            log.motor("fLTurn")
                                    .voltage(voltMut.mut_replace(fLTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(fLEncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(fLEncoder.getVelocity(), RadiansPerSecond));
                            log.motor("fRTurn")
                                    .voltage(voltMut.mut_replace(fRTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(fREncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(fREncoder.getVelocity(), RadiansPerSecond));
                            log.motor("rLTurn")
                                    .voltage(voltMut.mut_replace(rLTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(rLEncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(rLEncoder.getVelocity(), RadiansPerSecond));
                            log.motor("rRTurn")
                                    .voltage(voltMut.mut_replace(rRTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(rREncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(rREncoder.getVelocity(), RadiansPerSecond));
                    }, this));

        } else {
            DriverStation.reportError("Invalid motor type at SwerveDrivetrainMk4.getSysIdRoutine", false);
            return null;
        }
    }

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param motor The motor to run the test on either "drive" or "turn"
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(String motor, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(motor).quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param motor The motor to run the test on either "drive" or "turn"
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(String motor, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(motor).dynamic(direction);
    }
}




