package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
    private final Solenoid claw;
    public Grabber(PneumaticsControlModule pcm) {
        claw = pcm.makeSolenoid(Constants.GrabberConstants.GRABBER_ID);
        claw.set(false);
    }
    public void set(boolean setValue) {
        claw.set(setValue);
    }

    public boolean get() {
        return claw.get();
    }

    public Command setTrue() {return new InstantCommand(() -> this.set(true));}
    public Command setFalse() {return new InstantCommand(() -> this.set(false));}

}
