// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public final class TrajectoryFactory {
    public static final TrajectoryConfig defaultConfig = new TrajectoryConfig(
        Constants.Autonomous.kMaxVelocityMetersPerSecond, Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(Constants.Drivetrain.kDriveKinematics).addConstraint(Constants.Autonomous.kVoltageConstraint);

    public static Command getRamseteCommand(Drivetrain drivetrain, Trajectory trajectory) throws Exception {
        if (drivetrain == null || trajectory == null) {
            throw new Exception("Failed to get ramsete command: " + (drivetrain == null ? "drivetrain" : "trajectory") + " cannot be null!");
        }

        // Generate a ramsete command given the drivetrain and robot constraints
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory, 
            drivetrain::getPose, 
            new RamseteController(Constants.Autonomous.kRamseteBMetersPerSecond, Constants.Autonomous.kRamseteZetaMetersPerSecond), 
            new SimpleMotorFeedforward(
                Constants.Drivetrain.ksVolts,
                Constants.Drivetrain.kvVoltSecondsPerMeter,
                Constants.Drivetrain.kaVoltSecondsSquaredPerMeter
            ),
            Constants.Drivetrain.kDriveKinematics, 
            drivetrain::getWheelSpeeds, 
            new PIDController(Constants.Drivetrain.kpDriveVel, 0, 0),
            new PIDController(Constants.Drivetrain.kpDriveVel, 0, 0),
            drivetrain::tankDriveVolts,
            drivetrain
        );

        // Reset the odometry to the starting pose of the trajectory
        drivetrain.resetOdometry(trajectory.getInitialPose());

        // Returns the generated ramsete command, then stops the drivetrain on completion
        return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0.0, 0.0));
    }

    public static Trajectory getTrajectory(DrivePaths drivePath) {
        switch (drivePath) {
            case EXAMPLE_PATH:
                return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)), 
                    List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)
                    ),
                    new Pose2d(3, 0, new Rotation2d(0)),
                    defaultConfig
                );
        
            default:
                return null;
        }
    }

    public static enum DrivePaths {
        EXAMPLE_PATH,
    }
}
