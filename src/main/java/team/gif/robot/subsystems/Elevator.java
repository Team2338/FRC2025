package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.revrobotics.spark.config.LimitSwitchConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

import static com.ctre.phoenix6.signals.ForwardLimitSourceValue.RemoteTalonFX;


public class Elevator extends SubsystemBase {
    public final TalonFX elevatorMotor;

    public boolean elevatorManualFlag = false;
    private double elevatorTargetPos;

    String TalonFX = new String();
    BaseTalon BaseTalon = new BaseTalon(33,TalonFX);

    public Elevator() {
        elevatorMotor = new TalonFX(RobotMap.ELEVATOR_ID);
        configElevatorTalon();
        zeroEncoder();
    }

    /**
     * Moves the elevator with a percent output
     * @param percent percent of max speed to run the elevator motor
     */

    public void move(double percent) {
        final VoltageOut elevatorVoltage = new VoltageOut(0);
        elevatorMotor.setControl(elevatorVoltage.withOutput(percent));
    }

    /**
     * Holds the elevator at its given position with PID
     */
    public void PIDHold() {
        PositionDutyCycle elevatorPos = new PositionDutyCycle(0);
        elevatorMotor.setPosition(1,0);
        // the elevator needs a different kF when it is lower to the ground, otherwise it doesn't stay at the position
            //elevatorMotor.config_kF(1, Constants.Elevator.F_HOLD);
        elevatorMotor.setControl(elevatorPos.withPosition(elevatorTargetPos)); // closed loop position control
    }

    /**
     * Get the position of the elevator in encoder ticks
     * @return the position of the elevator in encoder ticks
     */
    public double getPosition() {
        PositionDutyCycle elevatorPos = new PositionDutyCycle(0);
        return elevatorPos.Position;
    }

    /**
     * Get the position of the elevator in inches
     * @return the position of the elevator in inches
     */
    public double getPositionInches() {
        PositionDutyCycle elevatorPos = new PositionDutyCycle(0);
        return (elevatorPos.Position + Constants.Elevator.ZERO_OFFSET_TICKS) / Constants.Elevator.EL_TICKS_PER_INCH;
    }

    /**
     * Convert a position in inches to encoder ticks
     * @param inches the position in inches
     * @return the position in encoder ticks
     */
    public double inchesToPos(int inches) {
        return inches * Constants.Elevator.EL_TICKS_PER_INCH - Constants.Elevator.ZERO_OFFSET_TICKS;
    }

    /**
     *
     * @param inches
     * @return Inches in units of ticks
     */
    public double inchesToTicks(int inches) {
        return inches * Constants.Elevator.EL_TICKS_PER_INCH;
    }

    /**
     * Get the target elevator position
     * @return the target elevator position
     */
    public double getTargetPosition() {
        return elevatorTargetPos;
    }

    /**
     * Get the PID error on the elevator position
     * @return the PID error on the elevator position
     */
    public double PIDError() {
        return Math.abs(getPosition() - elevatorTargetPos);
    }

    /**
     * Set the velocity of the elevator as a percent output
     * @param percent percent of max speed to run the elevator motor
     */
    public void setPercentOutput(double percent) {
        final DutyCycleOut elevatorPercentOutput = new DutyCycleOut(0);
        elevatorMotor.setControl(elevatorPercentOutput.withOutput(percent));
    }

    /**
     * Sets the elevator to a given position with Motion Magic
     * @param position the position to set the elevator to
     */
    public void setMotionMagic(double position) {
        final MotionMagicVoltage elevatorMotionMagic = new MotionMagicVoltage(0);
        elevatorMotor.setControl(elevatorMotionMagic.withPosition(position));
    }

    /**
     * Sets the cruise velocity of the elevator in ticks per 100ms
     * @param rotationsPersecond the cruise velocity of the elevator in Rotations Per second
     */
    public void setCruiseVelocity(int rotationsPersecond) {
        // new unit (rot/sec)
        elevatorMotor.getRotorVelocity();
    }

    /**
     * Sets the target elevator position
     * @param pos the target elevator position
     */
    public void setElevatorTargetPos(double pos) {
        elevatorTargetPos = pos;
    }

    /**
     * Sets the forward limit switch to be normally open or normally closed
     */
    public boolean getFwdLimit() {
        SensorCollection elSensor = new SensorCollection(BaseTalon);
        return elSensor.isFwdLimitSwitchClosed();
    }

    /**
     * Sets the reverse limit switch to be normally open or normally closed
     */
    public boolean getRevLimit() {
        SensorCollection elSensor = new SensorCollection(BaseTalon);
        return elSensor.isRevLimitSwitchClosed();
    }


    public boolean isFinished() {
        return  Math.abs(PIDError()) < Constants.Elevator.PID_TOLERANCE;
    }

    public double getOutputVoltage() {
        VoltageOut elevatorVolt = new VoltageOut(0);
        return elevatorVolt.Output;
    }

    /**
     * Gets the percent output of the elevator motor
     * @return the percent output of the elevator motor
     */
    public double getOutputPercent() {
        DutyCycleOut elevatorPercent = new DutyCycleOut(0);
        return elevatorPercent.Output;
    }

    /**
     *
     * Gets the velocity of the elevator in ticks per 100ms
     * @return the velocity of the elevator in ticks per 100ms
     */
    public double getVelTPS() {
        VelocityDutyCycle elVelocity = new VelocityDutyCycle(0);
        return elVelocity.Velocity * 10.0;
    }

    /**
     * Gets the current of the elevator motor
     */
    public double getCurrent() {
        TorqueCurrentFOC elCurrent = new TorqueCurrentFOC(0);
        return elCurrent.Output;
    }

    /**
     * Sets the reverse soft limit of the elevator
     */
    public void enableLowerSoftLimit(boolean engage) {
        elevatorMotor.getFault_ReverseSoftLimit(engage);
    }

    /**
     * Zeroes the elevator encoder
     */
    public void zeroEncoder() {
        PositionDutyCycle elPosition = new PositionDutyCycle(0);
        elPosition.withPosition(0);
    }

    /**
     * Configures the elevator Talon
     */
    private void configElevatorTalon() {
        elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
        //elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        //elevatorMotor.enableVoltageCompensation(true);
        //elevatorMotor.setSensorPhase(true);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        //creates configurable talon
        var talonFXConfigs = new TalonFXConfiguration();

        //applies configs to slot 0
        var elevatorConfigs = talonFXConfigs.Slot0;
        elevatorConfigs.kP = Constants.Elevator.ELEVATOR_KP;
        elevatorConfigs.kI = Constants.Elevator.ELEVATOR_KI;
        elevatorConfigs.kD = Constants.Elevator.ELEVATOR_KD;
        elevatorConfigs.kS = Constants.Elevator.ELEVATOR_KS;
        //apply configs with 50ms timeout
        elevatorMotor.getConfigurator().apply(elevatorConfigs, 0.05);

        //applies configs to slot 1
        var elevatorHoldConfigs = talonFXConfigs.Slot1;
        elevatorHoldConfigs.kP = Constants.Elevator.ELEVATOR_KP_HOLD;
        elevatorHoldConfigs.kI = Constants.Elevator.ELEVATOR_KI_HOLD;
        elevatorHoldConfigs.kD = Constants.Elevator.ELEVATOR_KD_HOLD;
        elevatorMotor.getConfigurator().apply(elevatorHoldConfigs, 0.05);

        MotionMagicConfigs elevatorMotionConfig = talonFXConfigs.MotionMagic;
        elevatorMotionConfig.withMotionMagicAcceleration(Constants.Elevator.MAX_ACCELERATION);
        elevatorMotionConfig.withMotionMagicCruiseVelocity(Constants.Elevator.MAX_VELOCITY);

        MotorOutputConfigs elevatorOutputConfigs = new MotorOutputConfigs();
        elevatorOutputConfigs.withPeakReverseDutyCycle(0);
        elevatorOutputConfigs.withPeakReverseDutyCycle(0);

        HardwareLimitSwitchConfigs elevatorHardwareSwitchConfigs = new HardwareLimitSwitchConfigs();
        elevatorHardwareSwitchConfigs.withForwardLimitSource(RemoteTalonFX);
        elevatorHardwareSwitchConfigs.withReverseLimitSource(ReverseLimitSourceValue.RemoteTalonFX);
        elevatorHardwareSwitchConfigs.withForwardLimitType(ForwardLimitTypeValue.NormallyClosed);
        elevatorHardwareSwitchConfigs.withReverseLimitType(ReverseLimitTypeValue.NormallyClosed);

        LimitSwitchConfig elevatorSwitchConfigs = new LimitSwitchConfig();
        elevatorSwitchConfigs.reverseLimitSwitchEnabled(true);
        elevatorSwitchConfigs.forwardLimitSwitchEnabled(true);



    }
}




