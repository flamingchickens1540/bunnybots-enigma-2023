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
    public static final int kCopilotControllerPort = 2;
    public static final int kPCM_ID = 25;
  }

  public static class DrivetrainConstants {
    public static final int FRONT_LEFT_KEY = 2;
    public static final int FRONT_RIGHT_KEY = 4;
    public static final int BACK_LEFT_KEY = 1;
    public static final int BACK_RIGHT_KEY = 3;
    public static final double ENCODER_TICKS_PER_METER = 1*2048;


    // to be clear, one of us needs to use SI to find these constants
    // they are not all just one, goofily
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 1;
    public static final double kaVoltSecondsSquaredPerMeter = 1;

    public static final double kPDriveVel = 1;

    public static final double kTrackwidthMeters = 1; // I am going to strangle fab & controls with my bare hands
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final int PIGEON_CAN_ID = 2;
  }

  public static class ElevatorConstants {
      public static final int MOTOR_ID = 5;
      public static final double ELEVATOR_KF = 0;
      public static final double ELEVATOR_KP = 0;
      public static final double ELEVATOR_KI = 0;
      public static final double ELEVATOR_KD = 0;
      public static final double ELEVATOR_CRUISE_VELOCITY = 0;
      public static final double ELEVATOR_ASCENSION_VELOCITY = 0.5;
      public static final double ELEVATOR_DESCENSION_VELOCITY = -0.5;
      public static final double ELEVATOR_MAX_ACCELERATION = 0;
      public static final double MANUAL_SCALING = 0.3;
      public static final double MANUAL_DEADZONE = 0.1;
  }

  public static class PneumaticsConstants {
    public static final int CLAW_ID = 0;

    public static final int SHOOTER_ID_1 = 1;
    public static final int SHOOTER_ID_2 = 2;
    public static final int SHOOTER_ID_3 = 3;
    public static final int SHOOTER_ID_4 = 4;
    public static final int SHOOTER_ID_5 = 5;
  }

  public static class ShooterConstants{
    public static final int[] SHOOTER_ID = new int[] {1, 3, 2, 5, 6};
    public static final int[] GUITAR_BUTTON_ID = new int[] {1, 2, 4, 3, 5};
    public static final double GUITAR_DONGLE_DEADZONE = 0.5;
  }

  public static class GrabberConstants{
    public static final int GRABBER_ID = 7;

  }

}
