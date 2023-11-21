package frc.robot.commands;

import com.pathplanner.lib.controllers.PPRamseteController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.Drivetrain;

public class AutoCommand extends SequentialCommandGroup {
    private Drivetrain m_Drivetrain;

    public AutoCommand(Drivetrain dt) {
        m_Drivetrain = dt;
        addRequirements(m_Drivetrain);
        

        PathPlannerPath path = PathPlannerPath.fromPathFile("RamAuto.path");
        PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(0,0,0));
        m_Drivetrain.resetOdometry(trajectory.getInitialDifferentialPose());

        PPRamseteController ramseteController = new PPRamseteController();
    }
}
