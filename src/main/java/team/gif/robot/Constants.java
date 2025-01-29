// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DEBOUNCE_DEFAULT = 0.020;

    public static final class Drivetrain { // ToDo tune - remove when done
        //public static final double DRIVE_WHEEL_RADIUS = 0.05; // meters? Must be unit of velocity

        public static final boolean kFrontLeftTurningEncoderReversed = false; //false
        public static final boolean kRearLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kRearRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kRearLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kRearRightDriveMotorReversed = false;

        public static final boolean kFrontLeftTurningMotorReversed = true;
        public static final boolean kRearLeftTurningMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kRearRightTurningMotorReversed = false;

        public static final double FRONT_LEFT_OFFSET = 79.641015625; //79.189; //82.089; // these are off by .1 values
        public static final double REAR_LEFT_OFFSET = -136.31835937; // -137.473046875; //221.66015;//-138.25195;
        public static final double FRONT_RIGHT_OFFSET = -20.578515625; //160.40039 + 180;// + 25.31;
        public static final double REAR_RIGHT_OFFSET = 155.590625; // 155.490625; // updated with new encoder 3/7/24

        // Distance between centers of front and back wheels on robot
        public static final double TRACK_LENGTH = Units.inchesToMeters(22.5);

        // Distance between centers of left and right wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);

        // location of wheels from center of robot using following axis
        //        +x
        //         ^
        //         |
        //  +y  <---
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2), // front left
                        new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2), // front right
                        new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2), // back left
                        new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)); // back right

        public static final boolean kGyroReversed = false;

        // TODO: get feedback from driver
        public static final double COAST_DRIVE_RPM = 2200; //3000;//2500; // 2750; //4800 demo speed //2750
        public static final double BOOST_DRIVE_RPM = 1100; // 1675 is max speed; was 1750;
        public static final double SLOW_DRIVE_RPM = 3500;

        public static final double COAST_SPEED_METERS_PER_SECOND = COAST_DRIVE_RPM *
                (Math.PI * team.gif.robot.Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * team.gif.robot.Constants.ModuleConstants.GEAR_RATIO);

        public static final double BOOST_SPEED_METERS_PER_SECOND = BOOST_DRIVE_RPM *
                (Math.PI * team.gif.robot.Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * team.gif.robot.Constants.ModuleConstants.GEAR_RATIO);

        public static final double SLOW_SPEED_METERS_PER_SECOND = SLOW_DRIVE_RPM *
                (Math.PI * team.gif.robot.Constants.ModuleConstants.WHEEL_DIAMETER_METERS) /
                (60.0 * team.gif.robot.Constants.ModuleConstants.GEAR_RATIO);
        public static double kMaxAccelerationMetersPerSecondSquared = 2;
    }

    public static final class ModuleConstants { // ToDo tune - remove when done
        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 6 * (2 * Math.PI); //6
        public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 6 * (2 * Math.PI); //7
        public static final double GEAR_RATIO = 153.0 / 25.0; //27.0 / 4.0; on 2023 bot. // need to ask luke
        public static final double ENCODER_CPR = 2048.0;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.78);
        public static final double DRIVE_ENCODER_ROT_2_METER = Math.PI * WHEEL_DIAMETER_METERS / (GEAR_RATIO * ENCODER_CPR);
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        //4096.0 for talons
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) ENCODER_CPR;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;

        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;
        public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TURNING_MOTOR_GEAR_RATIO = 1.0 / 18.0;

        public static final double TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;

        public static final double TURNING_ENCODER_RPM_2_RAD_PER_SECOND = TURNING_ENCODER_ROT_TO_RAD / 60;

        public static final class DrivetrainPID { //TODO: tuning pid
            public static final double frontLeftP = 0.55;// 0.35; //pBot 0.4 all P
            public static final double frontLeftFF = 0.01;//0.01; //pBot 0.01 all FF
            public static final double frontRightP = 0.55;
            public static final double frontRightFF = 0.01; //issa good
            public static final double rearLeftP = 0.55;//0.35;
            public static final double rearLeftFF = 0.01;//0.01;
            public static final double rearRightP = 0.55;//0.35; // 0.6
            public static final double rearRightFF = 0.01;//0.01;
        }
    }

    public static final class DrivetrainAuto {
        public static final double kP_FORWARD = 3.0;
        public static final double kP_ROTATION = 3.0;
        public static final double MAX_MODULE_SPEED_MPS = 4;
        public static final double DRIVEBASE_RADIUS_METERS = 0.4131;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 5.0;
        public static final double PY_CONTROLLER = 5.0;
        public static final double P_THETA_CONTROLLER = 3.7;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double DRIVE_SUPER_FAST = 1.0;
        public static final double DRIVE_FAST = 0.7;
        public static final double DRIVE_MEDIUM = 0.6;
        public static final double DRIVE_SLOW = 0.27; // 0.3;
        public static final double DRIVE_SUPER_SLOW = 0.2;
        public static final double HOLD_AT_ANGLE = 0.15;
        public static final double DRIVE_TIME_DEFAULT = 1.5; // seconds until the bot gets to the charging station
    }

    // copied this from 2023
    public static final class DrivetrainMK3 {
        //public static final double DRIVE_WHEEL_RADIUS = 0.05; // meters? Must be unit of velocity

        public static final boolean kFrontLeftTurningEncoderReversed = false; //false
        public static final boolean kRearLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kRearRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kRearLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kRearRightDriveMotorReversed = false;

        public static final boolean kFrontLeftTurningMotorReversed = false;
        public static final boolean kRearLeftTurningMotorReversed = true;
        public static final boolean kFrontRightTurningMotorReversed = true;
        public static final boolean kRearRightTurningMotorReversed = true;


        public static final double FRONT_LEFT_OFFSET = 42.8902;
        public static final double REAR_LEFT_OFFSET = 358.9453;
        public static final double FRONT_RIGHT_OFFSET =  255.4648;
        public static final double REAR_RIGHT_OFFSET = 199.0722;

        // Distance between centers of right and left wheels on robot
        public static final double TRACK_LENGTH = Units.inchesToMeters(22.5);

        // Distance between front and back wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(23);

        // location of wheels from center of robot using following axis
        //        +x
        //         ^
        //         |
        //  +y  <---
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2), // front left
                        new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2), // front right
                        new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2), // back left
                        new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)); // back right

        public static final boolean kGyroReversed = false;

        public static final double COAST_DRIVE_RPM = 2500; // 2750; //4800 demo speed //2750
        public static final double BOOST_DRIVE_RPM = 1675; // 1675 is max speed; was 1750;
        public static final double SLOW_DRIVE_RPM = 3500;

        public static final double COAST_SPEED_METERS_PER_SECOND = COAST_DRIVE_RPM *
                (Math.PI * ModuleConstantsMK3.WHEEL_DIAMETER_METERS) /
                (60.0 * ModuleConstantsMK3.GEAR_RATIO);

        public static final double BOOST_SPEED_METERS_PER_SECOND = BOOST_DRIVE_RPM *
                (Math.PI * ModuleConstantsMK3.WHEEL_DIAMETER_METERS) /
                (60.0 * ModuleConstantsMK3.GEAR_RATIO);

        public static final double SLOW_SPEED_METERS_PER_SECOND = SLOW_DRIVE_RPM *
                (Math.PI * ModuleConstantsMK3.WHEEL_DIAMETER_METERS) /
                (60.0 * ModuleConstantsMK3.GEAR_RATIO);
        public static double kMaxAccelerationMetersPerSecondSquared = 2;// TODO
    }

    public static final class ModuleConstantsMK3 {
        public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 6 * (2 * Math.PI); //6
        public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 6 * (2 * Math.PI); //7
        public static final double GEAR_RATIO = 6.86; // TODO: need to ask someone
        public static final double ENCODER_CPR = 42; //This is for DRIVE
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.78);
        public static final double WHEEL_CIRCUMFERENCE_METER = Units.inchesToMeters(12.875);
        public static final double DRIVE_ENCODER_ROT_2_METER = WHEEL_CIRCUMFERENCE_METER/ (GEAR_RATIO) * 0.98; //0.0004
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        //4096.0 for talons
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CPR;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) ENCODER_CPR;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;

        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;
        public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;

        public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;

        public static final double TURNING_MOTOR_GEAR_RATIO = 12.8; //

        //            public static final double TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        public static final double TURNING_ENCODER_ROT_TO_RAD = 2 * Math.PI / 4096;

        public static final double TURNING_ENCODER_RPM_2_RAD_PER_SECOND = TURNING_ENCODER_ROT_TO_RAD / 60;

        public static final class DrivetrainPID {
            public static final double frontLeftP = 0.35; //pBot 0.4 all P
            public static final double frontLeftFF = 0.01; //pBot 0.01 all FF
            public static final double frontRightP = 0.35;
            public static final double frontRightFF = 0.01; //issa good
            public static final double rearLeftP = 0.35;
            public static final double rearLeftFF = 0.01;
            public static final double rearRightP = 0.35; // 0.6
            public static final double rearRightFF = 0.01;
        }
    }

    public static final class Joystick {
        public static final double DEADBAND = 0.1;
    }
    public static final class MotorTemps {
        public static final double SHOOTER_MOTOR_TEMP = 70;
        public static final double INDEXER_MOTOR_TEMP = 70;
        public static final double DRIVETRAIN_MOTOR_TEMP = 85;
        public static final double COLLECTOR_MOTOR_TEMP = 70;
        public static final double ELEVATOR_MOTOR_TEMP = 70;
        public static final double CLIMBER_MOTOR_TEMP = 70;
    }
    public static final class LED {
        public static final int NUM_LEDS_TOTAL = 6;
    }

    public static final class SHOOTER {
        public static final double SHOOTER_SPEED_PERCENT = 0.50;
        public static final double SHOOTER_SPEED_INDEXER_PERCENT = 0.25;
    }
}