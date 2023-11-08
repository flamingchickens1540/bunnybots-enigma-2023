package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants;

// the funny part about me (Brandon) "writing" this for you, Simon
// is that this code is all yours

public class RamseteConfig {
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.DrivetrainConstants.kTrackwidthMeters);

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.DrivetrainConstants.ksVolts,
            Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
            Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter
        ),
        kDriveKinematics,
        10
    );

    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(kDriveKinematics).addConstraint(autoVoltageConstraint);

    public static final ChassisSpeeds getChassisSpeeds(DifferentialDriveWheelSpeeds diffSpeeds) {
        return kDriveKinematics.toChassisSpeeds(diffSpeeds);
    }

    public static final double[] getWheelSpeeds(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds diffSpeeds = kDriveKinematics.toWheelSpeeds(speeds);
        double leftVel = diffSpeeds.leftMetersPerSecond;
        double rightVel = diffSpeeds.rightMetersPerSecond;
        return new double[] {leftVel, rightVel};
    }


    public static final RamseteController ramseteController = new RamseteController(kRamseteB, kRamseteZeta);
    public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(
        Constants.DrivetrainConstants.ksVolts, 
        Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
        Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter);
}
