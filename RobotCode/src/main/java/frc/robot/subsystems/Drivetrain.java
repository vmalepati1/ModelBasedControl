package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AnalogChannels.GYRO_CHANNEL;
import static frc.robot.Constants.CANBusIDs.*;
import static frc.robot.Constants.DrivetrainConstants.*;

public class Drivetrain extends SubsystemBase {

    private Field2d field = new Field2d();

    private final CANSparkMax leftWheelsMaster = new CANSparkMax(DRIVETRAIN_LEFT_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax leftWheelsSlave = new CANSparkMax(DRIVETRAIN_LEFT_SLAVE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANSparkMax rightWheelsMaster = new CANSparkMax(DRIVETRAIN_RIGHT_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightWheelsSlave = new CANSparkMax(DRIVETRAIN_RIGHT_SLAVE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final SimDeviceSim leftWheelsMasterSim = new SimDeviceSim("SPARK MAX [3]");
    private final SimDeviceSim rightWheelsMasterSim = new SimDeviceSim("SPARK MAX [1]");

    private final AnalogGyro ahrs = new AnalogGyro(GYRO_CHANNEL);

    private final AnalogGyroSim ahrsSim = new AnalogGyroSim(ahrs);

    private final SimpleMotorFeedforward linearFeedforward = new SimpleMotorFeedforward(KsLinear, KvLinear, KaAngular);
    private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(getHeading());
    private final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(
            TRACK_WIDTH
    );
    private final RamseteController ramseteController = new RamseteController();

    private final PIDController leftVelocityPID = new PIDController(LEFT_VELOCITY_P, 0.0, 0.0);
    private final PIDController rightVelocityPID = new PIDController(RIGHT_VELOCITY_P, 0.0, 0.0);

    // Create the simulation model of our drivetrain.
    private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
            // Create a linear system from our characterization gains.
            LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
            DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
            10.0,
            TRACK_WIDTH,
            Units.inchesToMeters(5),

            // The standard deviations for measurement noise:
            // x and y:          0.001 m
            // heading:          0.001 rad
            // l and r velocity: 0.1   m/s
            // l and r position: 0.005 m
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    public Drivetrain() {
        leftWheelsMaster.restoreFactoryDefaults();
        leftWheelsSlave.restoreFactoryDefaults();
        rightWheelsMaster.restoreFactoryDefaults();
        rightWheelsSlave.restoreFactoryDefaults();

        leftWheelsMaster.setInverted(true);

        leftWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightWheelsSlave.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightWheelsMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftWheelsSlave.follow(leftWheelsMaster);
        rightWheelsSlave.follow(rightWheelsMaster);

        leftWheelsMaster.setOpenLoopRampRate(K_OPENLOOP_RAMP);
        leftWheelsSlave.setOpenLoopRampRate(K_OPENLOOP_RAMP);
        rightWheelsMaster.setOpenLoopRampRate(K_OPENLOOP_RAMP);
        rightWheelsSlave.setOpenLoopRampRate(K_OPENLOOP_RAMP);

        leftWheelsMaster.setSmartCurrentLimit(K_SMART_CURRENT_LIMIT);
        leftWheelsSlave.setSmartCurrentLimit(K_SMART_CURRENT_LIMIT);
        rightWheelsMaster.setSmartCurrentLimit(K_SMART_CURRENT_LIMIT);
        rightWheelsSlave.setSmartCurrentLimit(K_SMART_CURRENT_LIMIT);

        leftWheelsMaster.getEncoder().setPositionConversionFactor(POSITION_FACTOR);
        leftWheelsSlave.getEncoder().setPositionConversionFactor(POSITION_FACTOR);
        leftWheelsMaster.getEncoder().setPositionConversionFactor(POSITION_FACTOR);
        leftWheelsSlave.getEncoder().setPositionConversionFactor(POSITION_FACTOR);

        leftWheelsMaster.getEncoder().setVelocityConversionFactor(VELOCITY_FACTOR);
        leftWheelsSlave.getEncoder().setVelocityConversionFactor(VELOCITY_FACTOR);
        leftWheelsMaster.getEncoder().setVelocityConversionFactor(VELOCITY_FACTOR);
        leftWheelsSlave.getEncoder().setVelocityConversionFactor(VELOCITY_FACTOR);

        leftWheelsMaster.getPIDController().setP(VOLTAGE_P, 0);
        leftWheelsMaster.getPIDController().setI(0.0, 0);
        leftWheelsMaster.getPIDController().setD(0.0, 0);

        rightWheelsMaster.getPIDController().setP(VOLTAGE_P, 0);
        rightWheelsMaster.getPIDController().setI(0.0, 0);
        rightWheelsMaster.getPIDController().setD(0.0, 0);

        zeroAll();

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        updateRobotPose();
        field.setRobotPose(driveOdometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.
        driveSim.setInputs(leftWheelsMaster.get() * leftWheelsMaster.getBusVoltage(),
                rightWheelsMaster.get() * rightWheelsMaster.getBusVoltage());

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        driveSim.update(0.02);

        // Update all of our sensors.
        leftWheelsMasterSim.getDouble("Position").set(driveSim.getLeftPositionMeters());
//        System.out.println(driveSim.getLeftPositionMeters());
//        System.out.println(leftWheelsMaster.getEncoder().getPosition());
        leftWheelsMasterSim.getDouble("Velocity").set(driveSim.getLeftVelocityMetersPerSecond());
        rightWheelsMasterSim.getDouble("Position").set(driveSim.getRightPositionMeters());
        rightWheelsMasterSim.getDouble("Velocity").set(driveSim.getRightVelocityMetersPerSecond());
        ahrsSim.setAngle(-driveSim.getHeading().getDegrees());

//        System.out.println(leftWheelsMaster.getBusVoltage());
//        System.out.println(RobotController.getBatteryVoltage());
    }

    public Field2d getField() {
        return field;
    }

    public SimpleMotorFeedforward getLinearFeedforward() {
        return linearFeedforward;
    }

    public Pose2d getCurrentPose() {
        return driveOdometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                getLeftVelocityMetersPerSec(),
                getRightVelocityMetersPerSec()
        );
    }

    public double getLeftPositionMeters() {
        return leftWheelsMaster.getEncoder().getPosition();
    }

    public double getRightPositionMeters() {
        return rightWheelsMaster.getEncoder().getPosition();
    }

    public double getLeftVelocityMetersPerSec() {
        return leftWheelsMaster.getEncoder().getVelocity();
    }

    public double getRightVelocityMetersPerSec() {
        return rightWheelsMaster.getEncoder().getVelocity();
    }

    public double getLeftVoltage() {
        return leftWheelsMaster.getBusVoltage() * leftWheelsMaster.get();
    }

    public double getRightVoltage() {
        return rightWheelsMaster.getBusVoltage() * rightWheelsMaster.get();
    }

    private void updateRobotPose() {
        driveOdometry.update(getHeading(), getLeftPositionMeters(), getRightPositionMeters());
    }

    public DifferentialDriveKinematics getDriveKinematics() {
        return driveKinematics;
    }

    public RamseteController getRamseteController() {
        return ramseteController;
    }

    public Rotation2d getHeading() {
        return ahrs.getRotation2d();
    }

    public double getAngularVelocityDegreesPerSec() {
        return -ahrs.getRate();
    }

    public PIDController getLeftVelocityPID() {
        return leftVelocityPID;
    }

    public PIDController getRightVelocityPID() {
        return rightVelocityPID;
    }

    public void zeroAll() {
        zeroHeading();
        zeroEncoders();
    }

    public void zeroHeading() {
        if (RobotBase.isReal()) {
            ahrs.reset();
        } else {
            ahrsSim.setAngle(0.0);
        }
    }

    public void zeroEncoders() {
        if (RobotBase.isReal()) {
            leftWheelsMaster.getEncoder().setPosition(0.0);
            rightWheelsMaster.getEncoder().setPosition(0.0);
        } else {
            leftWheelsMasterSim.getDouble("Position").set(0.0);
            rightWheelsMasterSim.getDouble("Position").set(0.0);
        }
    }

    public void resetPose(Pose2d startingPose) {
        zeroEncoders();
        driveOdometry.resetPosition(startingPose, getHeading());
    }

    public void setDutyCycles(double leftDutyCycle, double rightDutyCycle) {
        leftWheelsMaster.set(leftDutyCycle);
        rightWheelsMaster.set(rightDutyCycle);
    }

    public void setVoltages(double leftVoltage, double rightVoltage) {
        leftWheelsMaster.set(leftVoltage / leftWheelsMaster.getBusVoltage());
        rightWheelsMaster.set(rightVoltage / rightWheelsMaster.getBusVoltage());
    }

}
