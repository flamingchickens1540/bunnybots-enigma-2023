package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorPosition extends CommandBase {
    private final Elevator elevator;
    private final double position;
    public ElevatorPosition(Elevator elevator, double position){
        this.elevator = elevator;
        this.position = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        //TODO: Wont actually work needs to have a range to end
        return elevator.getPosition() == position;
    }
}
