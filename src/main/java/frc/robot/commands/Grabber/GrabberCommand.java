package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberCommand extends CommandBase {
    private final Grabber grabber;
    public GrabberCommand(Grabber grabber) {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    public void initialize() {
        if (grabber.get()) {
            grabber.openGrabber();
        } else {
            grabber.closeGrabber();
        }
    }
    public boolean isFinished() {
        return grabber.get();
    }

}
