
// package frc.robot.commands;

// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.ShooterPivot;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Eyes;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Elevator;


// public class RunIntake extends Command {

//     // required subystems
//     Intake s_Intake;
//     ShooterPivot s_ShooterPivot;
//     Shooter s_Shooter;
//     Eyes s_Eyes;
//     Elevator s_Elevator;

//     // required WPILib class objects


//     // local variables




//     // constructor
//     public RunIntake(Intake s_Intake, ShooterPivot s_ShooterPivot, Shooter s_Shooter, Eyes s_Eyes, 
//     Elevator s_Elevator) {

//         this.s_Intake = s_Intake;
//         this.s_ShooterPivot = s_ShooterPivot;
//         this.s_Shooter = s_Shooter;
//         this.s_Eyes = s_Eyes;
//         this.s_Elevator = s_Elevator;

//         addRequirements(s_Intake, s_ShooterPivot, s_Shooter, s_Eyes, s_Elevator);


//     }

//     @Override
//     public void execute() {

//         s_Intake.setIntakePivotPosition(s_Intake.intakeGroundPosition);
//         s_Intake.setIntakeVoltage(s_Intake.runIntakeVoltage);
//         s_Elevator.SetElevatorPosition(0);
//         s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotIntakePosition);
//         s_Shooter.setLoaderVoltage(s_Shooter.runLoaderVoltage);
        
        
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {

//         s_Intake.setIntakePivotPosition(s_Intake.intakeSafePosition);
//         s_Intake.setIntakeVoltage(s_Intake.stopIntakeVoltage);
//         s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition);
//         s_Shooter.setLoaderVoltage(s_Shooter.stopLoaderVoltage);

//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return false; 
//     }
// }