package team.gif.robot;

public abstract class RobotMap {
    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;

    //SwerveDrivetrain
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 11; // Falcon 500 and Talon Fx
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 12;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 21;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 22;
    public static final int FRONT_LEFT_CANCODER_ID = 15;
    public static final int FRONT_RIGHT_CANCODER_ID = 25;
    public static final int REAR_LEFT_CANCODER_ID = 16;
    public static final int REAR_RIGHT_CANCODER_ID = 26;

    public static final int FRONT_LEFT_TURNING_MOTOR_ID = 13; //Neo and Spark
    public static final int REAR_LEFT_TURNING_MOTOR_ID = 14;
    public static final int FRONT_RIGHT_TURNING_MOTOR_ID = 23;
    public static final int REAR_RIGHT_TURNING_MOTOR_ID = 24;

    public static final int PIGEON_ID = 36;

    // MK3 RobotMap
    public static final int PRACTICE_REAR_LEFT_DRIVE_ID = 34;//1;
    public static final int PRACTICE_REAR_LEFT_TURN_ID = 9;//21;
    public static final int PRACTICE_REAR_RIGHT_DRIVE_ID = 1; //20;
    public static final int PRACTICE_REAR_RIGHT_TURN_ID = 32;//9;
    public static final int PRACTICE_FRONT_LEFT_DRIVE_ID = 14;//45;
    public static final int PRACTICE_FRONT_LEFT_TURN_ID = 7;//7;
    public static final int PRACTICE_FRONT_RIGHT_DRIVE_ID = 20;//14;
    public static final int PRACTICE_FRONT_RIGHT_TURN_ID = 31;//8;
    public static final int PIGEON_PBOT_ID = 61;
}
