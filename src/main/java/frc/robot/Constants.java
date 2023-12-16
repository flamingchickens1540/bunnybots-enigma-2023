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
  public static final double KP_DRIVE_VEL = 3.2925;
public static final double KS_VOLTS = 0.650;
public static final double KV_VOLT_SECONDS_PER_METER = 2.81;
public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.224;
public static final double ENCODER_TICKS_PER_METER = 49866;
  public static final double K_TRACKWIDTH_METERS = 0.6604;
  public static double DEADZONE = 0.1;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
    public static final int kPCM_ID = 25;
  }

  public static class DrivetrainConstants {
    public static final int FRONT_LEFT_KEY = 2;
    public static final int FRONT_RIGHT_KEY = 4;
    public static final int BACK_LEFT_KEY = 1;
    public static final int BACK_RIGHT_KEY = 3;
    public static final int PIGEON_CAN_ID = 7;

  }

  public static class ElevatorConstants {
      public static final int MOTOR_ID = 5;
      public static final double ELEVATOR_ASCENSION_VELOCITY = 0.9;
      public static final double ELEVATOR_DESCENSION_VELOCITY = -0.9;
      public static final double MANUAL_SCALING = 0.5;
      public static final double MANUAL_DEADZONE = 0.1;
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
