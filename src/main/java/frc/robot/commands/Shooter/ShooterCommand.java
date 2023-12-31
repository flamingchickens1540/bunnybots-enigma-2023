package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShooterCommand extends SequentialCommandGroup {


    public ShooterCommand(Shooter shooter, int index) {
        addCommands(
                shooter.getExtendCommand(index),
                new WaitCommand(0.3),
                shooter.getRetractCommand(index)
        );

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
