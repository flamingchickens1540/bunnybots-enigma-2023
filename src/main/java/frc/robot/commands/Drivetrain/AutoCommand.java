package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Elevator.Elevator;
import frc.robot.commands.Elevator.ElevatorDown;
import frc.robot.commands.Elevator.ElevatorUp;
import frc.robot.commands.Grabber.Grabber;
import frc.robot.commands.Grabber.GrabberCommand;

public class AutoCommand extends SequentialCommandGroup{
    public AutoCommand(Drivetrain drivetrain, String autoChoice, Elevator elevator, Grabber grabber){
        addRequirements(drivetrain);
        drivetrain.zeroHeading();

//        Pose2d start = new Pose2d(0,0,new Rotation2d(0));
//        Pose2d end = new Pose2d(3,0,new Rotation2d(Math.PI));

//        SmartDashboard.putNumber("pose/startPose", start.getRotation().getDegrees());
//        SmartDashboard.putNumber("pose/endPose", end.getRotation().getDegrees());

        PathPlannerTrajectory trajectory = null;
        if (autoChoice.equals("DoNothing")) {
            trajectory = new PathPlannerTrajectory();

            PPRamseteCommand ramseteCommand = new PPRamseteCommand(
                    trajectory,
                    drivetrain::getPose,
                    RamseteConfig.ramseteController,
                    RamseteConfig.feedForward,
                    RamseteConfig.kDriveKinematics,
                    drivetrain::getWheelSpeeds,
                    new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
                    new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
                    drivetrain::setVolts,
                    drivetrain);

            drivetrain.resetOdometry(trajectory.getInitialPose());

            addCommands(
                    ramseteCommand,
                    //      new InstantCommand(()-> SmartDashboard.putNumber("navx/endRotation", drivetrain.getPose().getRotation().getDegrees())), common Simon L
                    new InstantCommand(()-> drivetrain.setPercent(0, 0))
            );
        }
        else if (autoChoice.equals("RamAuto")) {
            trajectory = PathPlanner.loadPath("RamAuto", new PathConstraints(3.5, 2));

            PPRamseteCommand ramseteCommand = new PPRamseteCommand(
                    trajectory,
                    drivetrain::getPose,
                    RamseteConfig.ramseteController,
                    RamseteConfig.feedForward,
                    RamseteConfig.kDriveKinematics,
                    drivetrain::getWheelSpeeds,
                    new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
                    new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
                    drivetrain::setVolts,
                    drivetrain);

            drivetrain.resetOdometry(trajectory.getInitialPose());

            addCommands(
                    ramseteCommand,
                    //      new InstantCommand(()-> SmartDashboard.putNumber("navx/endRotation", drivetrain.getPose().getRotation().getDegrees())), common Simon L
                    new InstantCommand(()-> drivetrain.setPercent(0, 0))
            );
        }
        // need to add innout here
        else {
            trajectory = PathPlanner.loadPath("RamAuto", new PathConstraints(3.5, 2));

            PPRamseteCommand ramCommand = new PPRamseteCommand(
                    trajectory,
                    drivetrain::getPose,
                    RamseteConfig.ramseteController,
                    RamseteConfig.feedForward,
                    RamseteConfig.kDriveKinematics,
                    drivetrain::getWheelSpeeds,
                    new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
                    new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
                    drivetrain::setVolts,
                    drivetrain);

            ElevatorUp upCommand = new ElevatorUp(elevator);
            ElevatorDown downCommand = new ElevatorDown(elevator);
            GrabberCommand grabberCommand1 = new GrabberCommand(grabber);
            GrabberCommand grabberCommand2 = new GrabberCommand(grabber);

            drivetrain.resetOdometry(trajectory.getInitialPose());

            addCommands(
                ramCommand,
                upCommand,
                new InstantCommand(() -> grabber.set(!grabber.get())),
                new InstantCommand(() -> grabber.set(!grabber.get())),
                downCommand,
                new InstantCommand(() -> drivetrain.setPercent(0, 0))
            );
        }

    }
    

    
}