package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;

import static frc.robot.Robot.drivetrain;

public class Trajectories {

    public static Trajectory sCurve = generateSCurve();

    public static Trajectory generateSCurve() {
        TrajectoryConfig config = new TrajectoryConfig(
                Units.feetToMeters(4), Units.feetToMeters(3));

        config.setKinematics(drivetrain.getDriveKinematics());

        return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(
                        new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(13.5), Rotation2d.fromDegrees(0)),
                        new Pose2d(Units.feetToMeters(10.5), Units.feetToMeters(18.5), Rotation2d.fromDegrees(0))
                ),
                config
        );
    }

}
