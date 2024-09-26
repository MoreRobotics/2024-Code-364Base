
package frc.robot.commands;

import frc.robot.subsystems.Blower;
import edu.wpi.first.wpilibj2.command.Command;


public class Blow extends Command {

    // required subystems
    Blower s_Blower;

    // required WPILib class objects


    // local variables




    // constructor
    public Blow(Blower s_Blower) {

        this.s_Blower = s_Blower;

        addRequirements(s_Blower);


    }

    @Override
    public void initialize() {
        s_Blower.runBlower();
    }

    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Blower.stopBlower();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
}