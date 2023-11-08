package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Drivetrain.Drivetrain;

public class RamAutoCommand extends CommandBase {
    // not sure how to auto, will do later
    private Drivetrain m_Drivetrain;

    public RamAutoCommand(Drivetrain dt) {
        m_Drivetrain = dt;
        addRequirements(m_Drivetrain);
    }

    
}
