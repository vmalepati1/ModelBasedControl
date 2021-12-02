package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.ElevatorCommand;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.ElevatorChannels.ELEVATOR_LIMIT_LOWER_CHANNEL;
import static frc.robot.Constants.ElevatorChannels.ELEVATOR_MOTOR_CHANNEL;

public class Elevator extends Subsystem {

    private final TalonSRX elevatorMotor = new TalonSRX(ELEVATOR_MOTOR_CHANNEL);
    private final DigitalInput elevatorLimitLower = new DigitalInput(ELEVATOR_LIMIT_LOWER_CHANNEL);

    //	private final ElevatorLogger logger;
    private final Timer timer = new Timer();
    private boolean isMotionControlled;
    private double power;
    private boolean zeroing;
    private boolean zeroed;
    private double zeroStartTime;

    public Elevator() {
        zeroing = true;
//		zeroEncoder();
        initConstants();
//		this.logger = logger;
    }

    public double getError() {
        return isMotionControlled ? elevatorMotor.getClosedLoopError(0) : 0;
    }

    @Override
    protected void initDefaultCommand() {
        // TODO: Add back!
//        setDefaultCommand(new ElevatorCommand());
    }

    @Override
    public void periodic() {
//		logger.addMotionData(
//			new ElevatorData(Timer.getFPGATimestamp(), getCurrentPosition(),
//				getCurrentPositionNU(), power));

        if (zeroing) {
            setPower(-0.2);

            if (!elevatorLimitLower.get() || (Timer.getFPGATimestamp() - zeroStartTime > 1.0)) {
                zeroing = false;
                timer.stop();
                enableControl();
                zeroEncoder();
            }
        }
    }

    public void initConstants() {
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TIMEOUT);
        elevatorMotor.setSensorPhase(Robot.getChoosenRobot().getSensorPhase()); // true for comp bot; false for practice
        elevatorMotor.setInverted(Robot.getChoosenRobot().isReversed() /*false*/);

        elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT);
        elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);

        elevatorMotor.configNominalOutputForward(0, TIMEOUT);
        elevatorMotor.configNominalOutputReverse(0, TIMEOUT);
        elevatorMotor.configPeakOutputForward(1, TIMEOUT);
        elevatorMotor.configPeakOutputReverse(-1, TIMEOUT);

        elevatorMotor.selectProfileSlot(0, 0);

        elevatorMotor.config_kP(0, KP, TIMEOUT);
        elevatorMotor.config_kI(0, KI, TIMEOUT);
        elevatorMotor.config_kD(0, KD, TIMEOUT);
        elevatorMotor.config_kF(0, KF, TIMEOUT);

        elevatorMotor.configMotionCruiseVelocity(CRUISE_SPEED, TIMEOUT);
        elevatorMotor.configMotionAcceleration(ACCELERATION, TIMEOUT);

        zeroed = false;

        elevatorMotor.configForwardSoftLimitThreshold(Robot.getChoosenRobot().getMaximumElevatorHeight(), TIMEOUT);
        elevatorMotor.configReverseSoftLimitThreshold(Robot.getChoosenRobot().getMinimumElevatorHeight(), TIMEOUT);

        elevatorMotor.configForwardSoftLimitEnable(true, 10);
        elevatorMotor.configReverseSoftLimitEnable(false, 10);

        /* pass false to FORCE OFF the feature.  Otherwise the enable flags above are honored */
        elevatorMotor.overrideLimitSwitchesEnable(true);
    }

    public void zeroEncoder() {
        elevatorMotor.setSelectedSensorPosition(0, 0, TIMEOUT); // there was a to-do(figure out later) here

        elevatorMotor.configForwardSoftLimitThreshold(Robot.getChoosenRobot().getMaximumElevatorHeight(), 10);
        elevatorMotor.configReverseSoftLimitThreshold(Robot.getChoosenRobot().getMinimumElevatorHeight(), 10);

        elevatorMotor.configForwardSoftLimitEnable(true, 10);
        elevatorMotor.configReverseSoftLimitEnable(true, 10);

        /* pass false to FORCE OFF the feature.  Otherwise the enable flags above are honored */
        elevatorMotor.overrideLimitSwitchesEnable(true);

        zeroed = true;
    }

    public void nudge(double distance) {
        if (isMotionControlled && zeroed) {
            setTarget(getCurrentPositionInches() + distance);
        }
    }

    public void enableControl() {
        if (zeroed) {
            isMotionControlled = true;
            elevatorMotor.set(ControlMode.MotionMagic, elevatorMotor.getSelectedSensorPosition(0));
        }
    }

    public void disableControl() {
        isMotionControlled = false;
        elevatorMotor.set(ControlMode.Disabled, 0);
    }

    public boolean isZeroed() {
        return zeroed;
    }

    public boolean isZeroing() {
        return zeroing;
    }

    public void zero() {
        zeroing = true;
        zeroed = false;
        elevatorMotor.configReverseSoftLimitEnable(false, 10);
//            zeroEncoder();
        disableControl();

//			System.out.println("Heloo ");
        zeroStartTime = Timer.getFPGATimestamp();
    }

    public boolean isMotionControlled() {
        return isMotionControlled;
    }

    /**
     * Gets the current position of the elevator in inches.
     */
    public double getCurrentPositionInches() {
        return elevatorMotor.getSelectedSensorPosition(0) / Robot.getChoosenRobot()
                .getInchesToNativeUnits();
    }

    public double getCurrentVelocityInchesPerSec() {
        return elevatorMotor.getSelectedSensorVelocity(0) / Robot.getChoosenRobot()
                .getInchesToNativeUnits() * 10;
    }

    public double getAppliedVoltage() {
        return elevatorMotor.getMotorOutputVoltage();
    }

    /**
     * Gets the current position of the elevator in native units
     */
    public double getCurrentPositionNU() {
        return elevatorMotor.getSelectedSensorPosition(0);
    }

    /**
     * Sets the target position from the zero point.
     *
     * @param inches the position is in inches, duh
     */
    public void setTarget(double inches) {
//		System.out.println("Target: " + inches);

        if (zeroed) {
            elevatorMotor.set(
                    ControlMode.MotionMagic,
                    Math.min(Robot.getChoosenRobot().getMaximumElevatorHeight(),
                            Math.max(Robot.getChoosenRobot().getMinimumElevatorHeight(),
                                    inches * Robot.getChoosenRobot()
                                            .getInchesToNativeUnits())) /* This is a hard cap */
            );
        }
    }

    public void setPower(double percent) {
//		percent = Math.min(0.75, Math.max(-0.75, percent)); // throttle power in

        power = percent;
        SmartDashboard.putNumber("Elevator Power", percent);

//		System.out.println("Setting Power "  + percent);
        elevatorMotor.set(ControlMode.PercentOutput, percent);
    }

    public boolean atTopPosition() {
        return getCurrentPositionNU() >= Robot.getChoosenRobot().getMaximumElevatorHeight();
    }

    public boolean atLowerPosition() {
        return getCurrentPositionNU() <= Robot.getChoosenRobot().getMinimumElevatorHeight();
    }

    public void startZero() {
        if (!zeroed) {
            zero();
        }
    }
}

