package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class ElevatorManual extends CommandBase {
    private final Elevator elevator;
    private CommandXboxController controller;
    public ElevatorManual(Elevator elevator, CommandXboxController controller){
        this.elevator = elevator;
        this.controller = controller;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double positionMod = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
        positionMod *= Constants.ElevatorConstants.MANUAL_SCALING;
        positionMod = MathUtil.applyDeadband(positionMod, Constants.ElevatorConstants.MANUAL_DEADBAND);
        elevator.setPosition(elevator.getPosition() + positionMod);
    }
}
