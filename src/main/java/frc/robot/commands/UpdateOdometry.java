package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class UpdateOdometry extends Command {    
    private Swerve s_Swerve;    
    private Eyes s_Eyes;

    public UpdateOdometry(Swerve s_Swerve, Eyes s_Eyes) {
        this.s_Swerve = s_Swerve;
        this.s_Eyes = s_Eyes;
        addRequirements(s_Swerve, s_Eyes);

    }

    @Override
    public void execute() {

        s_Swerve.setPose(s_Eyes.getRobotPose());
        
    }
}