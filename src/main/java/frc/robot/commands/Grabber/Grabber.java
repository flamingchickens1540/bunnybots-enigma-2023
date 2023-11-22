package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
    private final Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.GrabberConstants.RIGHT_GRABBER_ID);
    private final Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.GrabberConstants.LEFT_GRABBER_ID);


    public void openGrabber() {
        rightSolenoid.set(false);
        leftSolenoid.set(false);
    }

    public void closeGrabber() {
        rightSolenoid.set(true);
        leftSolenoid.set(true);
    }

    public boolean get() {
        return rightSolenoid.get();
    }



}
