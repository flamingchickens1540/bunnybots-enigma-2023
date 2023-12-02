package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
    private final Solenoid claw;
    public Grabber(PneumaticsControlModule pcm) {
        claw = pcm.makeSolenoid(Constants.GrabberConstants.GRABBER_ID);
    }
    public void openGrabber() {
        claw.set(false);
    }

    public void closeGrabber() {
        claw.set(true);

    }

    public boolean get() {
        return claw.get();
    }



}
