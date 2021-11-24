package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.AnalogChannels.GYRO_CHANNEL;
import static frc.robot.Constants.CANBusIDs.*;
import static frc.robot.Constants.DrivetrainConstants.K_OPENLOOP_RAMP;
import static frc.robot.Constants.DrivetrainConstants.K_SMART_CURRENT_LIMIT;

public class Drivetrain extends SubsystemBase {

    private final CANSparkMax leftWheelsMaster = new CANSparkMax(DRIVETRAIN_LEFT_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax leftWheelsSlave = new CANSparkMax(DRIVETRAIN_LEFT_SLAVE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANSparkMax rightWheelsMaster = new CANSparkMax(DRIVETRAIN_RIGHT_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightWheelsSlave = new CANSparkMax(DRIVETRAIN_RIGHT_SLAVE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final SimDeviceSim leftWheelsMasterSim = new SimDeviceSim("SPARK MAX", DRIVETRAIN_LEFT_MASTER_ID);
    private final SimDeviceSim rightWheelsMasterSim = new SimDeviceSim("SPARK MAX", DRIVETRAIN_RIGHT_MASTER_ID);

    private final AnalogGyro ahrs = new AnalogGyro(GYRO_CHANNEL);

    private final AnalogGyroSim ahrsSim = new AnalogGyroSim(ahrs);

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

        ahrs.reset();
    }

    @Override
    public void periodic() {

    }

    public void setDutyCycles(double leftDutyCycle, double rightDutyCycle) {
        leftWheelsMaster.set(leftDutyCycle);
        rightWheelsMaster.set(rightDutyCycle);
    }

}
