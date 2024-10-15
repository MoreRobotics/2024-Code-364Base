
// package frc.robot.commands;

// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.ShooterPivot;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Eyes;
// import edu.wpi.first.wpilibj2.command.Command;


// public class PrepareShot extends Command {

//     // required subystems
//     ShooterPivot s_ShooterPivot;
//     Shooter s_Shooter;
//     Eyes s_Eyes;

//     // required WPILib class objects


//     // local variables
//     boolean shotReady = false;
//     boolean onMove = false;



//     // constructor
//     public PrepareShot(ShooterPivot s_ShooterPivot, Shooter s_Shooter, Eyes s_Eyes, boolean onMove) {

//         this.s_ShooterPivot = s_ShooterPivot;
//         this.s_Shooter = s_Shooter;
//         this.s_Eyes = s_Eyes;
//         this.onMove = onMove;

//         addRequirements(s_ShooterPivot, s_Shooter, s_Eyes);


//     }

//     @Override
//     public void execute() {
//         if (s_ShooterPivot.atPosition() == true && s_Eyes.swerveAtPosition(onMove) == true && s_Shooter.isUpToSpeed() == true) {
//             shotReady = true;
//         } else {
//             shotReady = false; 
//         }
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {}

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {

//         return shotReady;
        
//     }
// }