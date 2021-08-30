// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain {
        public static final double kWheelDiameterInches = 2.75591; // 70 mm
        // Horizontal distance between the wheels in meters
        public static final double kTrackwidthMeters = 0.149;

        // TODO: CALCULATE ALL VALUES WITH ZEROS IN THEM
        public static final double ksVolts = 0.0;
        public static final double kvVoltSecondsPerMeter = 0.0;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0;

        public static final double kpDriveVel = 0.0;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kEncoderTicksPerRevolution = 1440.0;
        public static final double kEncoderDistancePerPulse = Math.PI * kWheelDiameterInches / kEncoderTicksPerRevolution;
    }

    public static final class Autonomous {
        public static final double kMaxVoltage = 10;

        public static final double kMaxVelocityMetersPerSecond = 0.75;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.3;

        public static final double kRamseteBMetersPerSecond = 2;
        public static final double kRamseteZetaMetersPerSecond = 0.7;

        public static final DifferentialDriveVoltageConstraint kVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.Drivetrain.ksVolts,
                Constants.Drivetrain.kvVoltSecondsPerMeter,
                Constants.Drivetrain.kaVoltSecondsSquaredPerMeter
            ),
            Constants.Drivetrain.kDriveKinematics,
            kMaxVoltage
        );
    }
}
