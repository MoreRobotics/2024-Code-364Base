// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Eyes;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.ShooterPivot;


// public class PrepareFeed extends Command { 
//   private Shooter shooter;

//   private double shooterAngle;
//   private double shooterSpeed;

//   public PrepareFeed(Shooter shooter, double shooterSpeed) {

//     // Use addRequirements() here to declare subsystem dependencies.
//     this.shooter = shooter;
//     this.shooterSpeed = shooterSpeed;
//     addRequirements(shooter);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {} 

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     shooter.shootingMotorsSetControl(shooterSpeed, shooterSpeed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
