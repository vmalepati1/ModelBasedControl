// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.cscore.CameraServerCvJNI;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;

import java.util.ArrayList;

import static frc.robot.Constants.Elevator.TOLERANCE;
import static frc.robot.OI.gamepad;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  String data = "";

  int counter = 0;
  double startTime = 0;
  double priorAutospeed = 0;

  double[] numberArray = new double[10];
  ArrayList<Double> entries = new ArrayList<Double>();

  public Robot() {
    super(0.005);
    LiveWindow.disableAllTelemetry();
  }

  public static Elevator elevator;

  private static Constants.Robot currentRobot;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    currentRobot = Constants.Robot.COMPETITION;

    elevator = new Elevator();

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();

    SmartDashboard.putNumber("l_encoder_pos", elevator.getCurrentPositionInches());
    SmartDashboard.putNumber("l_encoder_rate", elevator.getCurrentPositionNU());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
    startTime = Timer.getFPGATimestamp();
    counter = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition =  elevator.getCurrentPositionInches();
    double leftRate = elevator.getCurrentVelocityInchesPerSec();

    double rightPosition = leftPosition;
    double rightRate = leftRate;

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    elevator.setPower(autospeed);

    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = 0.0;

    // Add data to a string that is uploaded to NT
    for (double num : numberArray) {
      entries.add(num);
    }
    counter++;
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (Math.abs(gamepad.getLeftY()) > TOLERANCE) {
      double power = -gamepad.getLeftY();

      if (power < 0) {
        power *= 0.75;
      }

      elevator.setPower(power);
    } else if (Math.abs(gamepad.getRightY()) > TOLERANCE) {
      double power = -gamepad.getRightY();

      if (power < 0) {
        power *= 0.75;
      }

      elevator.setPower(power);
    } else {
      elevator.setPower(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Robot disabled");
    elevator.setPower(0.0);
    // data processing step
    data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    entries.clear();
    System.out.println("Robot disabled");
    System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
    data = "";
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public static Constants.Robot getChoosenRobot() {
    return currentRobot;
  }
}
