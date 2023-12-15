package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final Solenoid[] shooters = new Solenoid[5];

    public Shooter(PneumaticsControlModule pcm) {
        for (int i = 0; i < 5; i++) {
            shooters[i] = pcm.makeSolenoid(Constants.ShooterConstants.SHOOTER_ID[i]);
        }
    }


    public Command getExtendCommand(int index) {
        return new InstantCommand(() -> {shooters[index].set(true);});
    }

    public Command getRetractCommand(int index) {
        return new InstantCommand(() -> {shooters[index].set(false);});
    }
}
