
package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;


public class RunLoader extends Command {

    // required subystems
    Shooter s_Shooter;

    // required WPILib class objects


    // local variables




    // constructor
    public RunLoader(Shooter s_Shooter) {

        this.s_Shooter = s_Shooter;

        addRequirements(s_Shooter);


    }

    @Override
    public void execute() {

        s_Shooter.setLoaderVoltage(s_Shooter.runLoaderVoltage);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        s_Shooter.setLoaderVoltage(s_Shooter.stopLoaderVoltage);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
}