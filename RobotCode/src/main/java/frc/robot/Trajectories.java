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

    public static Trajectory forwards5ft = generateForwards5ft();

    public static Trajectory generateForwards5ft() {
        TrajectoryConfig config = new TrajectoryConfig(
                Units.feetToMeters(4), Units.feetToMeters(3));

        config.setKinematics(drivetrain.getDriveKinematics());

        return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(new Pose2d(Units.feetToMeters(10.0), Units.feetToMeters(13.0),
                                Rotation2d.fromDegrees(0.0)),
                        new Pose2d(Units.feetToMeters(15.0), Units.feetToMeters(13.0),
                                Rotation2d.fromDegrees(0.0))),
                config
        );
    }

}
