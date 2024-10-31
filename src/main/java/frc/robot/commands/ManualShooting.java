// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.ShooterPivot;

// public class ManualShooting extends Command {
//   private Shooter shooter;
//   private ShooterPivot shooterPivot;
//   /** Creates a new ManualShooting. */
//   public ManualShooting( Shooter shooter, ShooterPivot shooterPivot) {
//     this.shooter = shooter;
//     this.shooterPivot = shooterPivot;

//     addRequirements(shooter, shooterPivot);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double angle = SmartDashboard.getNumber("Shooter Angle Input", 0);
//     shooterPivot.moveShooterPivot(angle);
//     double speed = SmartDashboard.getNumber("Shooter Speed Input", 0);
//     shooter.shootingMotorsSetControl(speed, speed);
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
