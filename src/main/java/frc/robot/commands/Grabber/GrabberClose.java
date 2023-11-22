package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberClose extends CommandBase {
    private final Grabber grabber;
    public GrabberClose(Grabber grabber) {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    public void initialize() {
        grabber.closeGrabber();
    }
    public boolean isFinished() {
        return grabber.get();
    }

}
