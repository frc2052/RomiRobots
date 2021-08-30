// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.TrajectoryFactory.DrivePaths;
import frc.robot.subsystems.Drivetrain;

public class ExampleAutoPath extends SequentialCommandGroup {
  public ExampleAutoPath(Drivetrain drivetrain) {
    try {
      addCommands(
        TrajectoryFactory.getRamseteCommand(drivetrain, TrajectoryFactory.getTrajectory(DrivePaths.EXAMPLE_PATH))
      );
    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    }
  }
}
