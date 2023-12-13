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

public class AutoCommand extends SequentialCommandGroup{
    public AutoCommand(Drivetrain drivetrain){
        addRequirements(drivetrain);
        drivetrain.zeroHeading();

        Pose2d start = new Pose2d(0,0,new Rotation2d(0));
        Pose2d end = new Pose2d(3,0,new Rotation2d(180));

        SmartDashboard.putNumber("pose/startPose", start.getRotation().getDegrees());
        SmartDashboard.putNumber("pose/endPose", end.getRotation().getDegrees());

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("backwardsPath", new PathConstraints(3.5, 2));

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
            drivetrain
        );


        drivetrain.resetOdometry(trajectory.getInitialPose());
        

        addCommands(
            ramseteCommand,
            new InstantCommand(()-> SmartDashboard.putNumber("navx/endRotation", drivetrain.getPose().getRotation().getDegrees())),
            new InstantCommand(()-> drivetrain.setVolts(0, 0))
        );
    }
    

    
}