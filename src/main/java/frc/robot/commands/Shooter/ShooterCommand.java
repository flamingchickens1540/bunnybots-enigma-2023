package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShooterCommand extends SequentialCommandGroup {


    public ShooterCommand(Shooter shooter, int index) {

        addRequirements(shooter);
        addCommands(
                shooter.getExtendCommand(index),
                new WaitCommand(0.3),
                shooter.getRetractCommand(index)
        );


    }
}
