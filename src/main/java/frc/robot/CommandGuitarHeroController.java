package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandGuitarHeroController extends CommandGenericHID{
    private final GenericHID m_hid;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public CommandGuitarHeroController(int port) {
      super(port);
      m_hid = new GenericHID(port);
    }

    public Trigger button1(){
        return new Trigger(() -> {
            return m_hid.getRawButtonPressed(1);
        });
    }

    public Trigger button2(){
        return new Trigger(() -> {
            return m_hid.getRawButtonPressed(2);
        });
        
    }

    public Trigger button3(){
        return new Trigger(() -> {
            return m_hid.getRawButtonPressed(3);
        });
    }

    public Trigger button4(){
        return new Trigger(() -> {
            return m_hid.getRawButtonPressed(4);
        });
    }

    public Trigger button5(){
        return new Trigger(() -> {
            return m_hid.getRawButtonPressed(5);
        });
    }

    public Trigger povUp(){
        return new Trigger(() -> {
            return m_hid.getPOV(0) == 0;
        });
    }

    public Trigger povDown(){
        return new Trigger(() -> {
            return m_hid.getPOV(0) == 180;
        });
    }

    public double rawAxis(){
        return m_hid.getRawAxis(4);
    }

    public Trigger triggerAxis(){
        return new Trigger(() -> {
            return m_hid.getRawAxis(4) > 0.95;
        });
    }

    public Trigger button7(){
        return new Trigger(() -> {
            return m_hid.getRawButtonPressed(7);
        });
    }

    public Trigger button8(){
        return new Trigger(() -> {
            return m_hid.getRawButtonPressed(8);
        });
    }
}