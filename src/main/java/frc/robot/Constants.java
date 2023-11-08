// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 //TODO Find the correct value for all these constants
public final class Constants {
  public static double DEADZONE = 0.1;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    public static final int FRONT_LEFT_KEY = 0;
    public static final int FRONT_RIGHT_KEY = 0;
    public static final int BACK_LEFT_KEY = 0;
    public static final int BACK_RIGHT_KEY = 0;
    public static final double ENCODER_TICKS_PER_METER = 1;


    // to be clear, one of us needs to use SI to find these constants
    // they are not all just one, goofily
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 1;
    public static final double kaVoltSecondsSquaredPerMeter = 1;

    public static final double kPDriveVel = 1;

    public static final double kTrackwidthMeters = 1; // I am going to strangle fab with my bare hands
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final int PIGEON_CAN_ID = 2;


  }
}
