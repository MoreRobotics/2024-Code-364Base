package frc.robot;

import java.sql.Driver;
import java.time.Instant;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    double rotationSpeed = 1.0;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    /* Drive Controls */
    private final int leftY = XboxController.Axis.kLeftY.value;
    private final int leftX = XboxController.Axis.kLeftX.value;
    private final int rightX = XboxController.Axis.kRightX.value;
    /*Operator Buttons */
    private final JoystickButton operatorButtonY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorButtonA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorButtonB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton operatorButtonX = new JoystickButton(operator, XboxController.Button.kX.value);
    /* Driver Buttons */
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverLB = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverRB = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driverLStick = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton driverRStick = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driverSelect = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final POVButton driverDpadUp = new POVButton(driver, 0);
    private final POVButton driverDpadRight = new POVButton(driver, 90);
    private final POVButton driverDpadDown = new POVButton(driver, 180);
    private final POVButton driverDpadLeft = new POVButton(driver, 270);

    /* driver axis */
    private final int driverLeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
    private final int driverRightTriggerAxis = XboxController.Axis.kRightTrigger.value;
    
    /* driver triggers */
    final Trigger driverLeftTrigger = new Trigger(() -> driver.getRawAxis(driverLeftTriggerAxis) > 0.1);
    final Trigger driverRightTrigger = new Trigger(() -> driver.getRawAxis(driverRightTriggerAxis) > 0.1);
 
    /* Operator Buttons */
    private final JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorLB = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton operatorRB = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton operatorLStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton operatorRStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton operatorUpStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton operatorDownStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);

    private final JoystickButton operatorStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton operatorBack = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final POVButton operatorDpadUp = new POVButton(operator, 0);
    private final POVButton operatorDpadRight = new POVButton(operator, 90);
    private final POVButton operatorDpadDown = new POVButton(operator, 180);
    private final POVButton operatorDpadLeft = new POVButton(operator, 270);

    /* operator axis */
    private final int operatorLeftYAxis = XboxController.Axis.kLeftY.value;
    private final int operatorRightYAxis = XboxController.Axis.kRightY.value;
    private final int operatorLeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
    private final int operatorRightTriggerAxis = XboxController.Axis.kRightTrigger.value;

    /* operator triggers */
    final Trigger operatorLeftTrigger = new Trigger(() -> operator.getRawAxis(operatorLeftTriggerAxis) > 0.1);
    final Trigger operatorRightTrigger = new Trigger(() -> operator.getRawAxis(operatorRightTriggerAxis) > 0.1);

    
    
    /* Subsystems */
    public final ShooterPivot s_ShooterPivot = new ShooterPivot();
    private final Swerve s_Swerve = new Swerve();
    private final Elevator s_Elevator = new Elevator();
    private final Intake s_Intake = new Intake();
    public final Shooter s_Shooter = new Shooter();
    private final Eyes s_Eyes = new Eyes(s_Swerve, s_Shooter);
    private final Blower s_Blower = new Blower();

    /* Commands */
    private final SendableChooser<Command> autoChooser;
    



    /** The container for the robot. Contains subsystems, OI devices, and commands. */

    public RobotContainer() {

    //Default Commands 
        //Swerve Drive
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(leftY), 
                () -> -driver.getRawAxis(leftX), 
                () -> driver.getRawAxis(rightX),
                () -> false,
                () -> s_Swerve.getGyroYaw().getDegrees(),
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                false
            )
        );
        
        } else {
        s_Swerve.setPose(new Pose2d(16.54, 0, new Rotation2d(Math.PI)));
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(leftY), 
                () -> driver.getRawAxis(leftX), 
                () -> driver.getRawAxis(rightX),
                () -> false,
                () -> s_Swerve.getGyroYaw().getDegrees(),
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                false
            )
        );

        }

        //spin up shooter when we have a note in the indexer
        //NOTE: IF REMOVED NEED TO ADD SHOOTER SPINUP FOR AMP SCORE
        s_Shooter.setDefaultCommand(
            new ConditionalCommand(
                new InstantCommand (() -> s_Shooter.setShooterVoltage(0, 0), s_Shooter), 
                new InstantCommand(() -> s_Shooter.shootingMotorsSetControl(35.0, 35.0), s_Shooter), //s_Shooter.shootingMotorsSetControl(20, 20)
                () -> s_Shooter.getBreakBeamOutput())
        );

        // Configure the button bindings
        configureButtonBindings();


        //Auto Commands

        Command AimThenShootAuto = new ParallelRaceGroup(
            
            new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter, false, false, false).alongWith(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> 0, 
                    () -> 0, 
                    () -> 0,
                    () -> false,
                    () -> s_Eyes.getTargetRotation(),
                    () -> true,
                    rotationSpeed,
                    true
                )
            ),
            
            new SequentialCommandGroup(
                new WaitCommand(2.0).until(() -> prepareShot()),
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.runLoaderVoltage)), 
                new WaitCommand(1.5)).until(() -> s_Shooter.getBreakBeamOutput())
                
                );



        NamedCommands.registerCommand("Intake", new RunIntake(s_Intake, s_ShooterPivot, s_Shooter, s_Eyes, s_Elevator)
            .until(() -> !s_Shooter.getBreakBeamOutput()) //TODO Make rollers spin after at position/0.25s
        );
        NamedCommands.registerCommand("Confirm Intake", new RunIntake(s_Intake, s_ShooterPivot, s_Shooter, s_Eyes, s_Elevator)
            .until(() -> !s_Shooter.getBreakBeamOutput()) //TODO Make rollers spin after at position/0.25s
            .withTimeout(1.25)
        );
        NamedCommands.registerCommand("AutoScore", AimThenShootAuto);
        NamedCommands.registerCommand("Aim", new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter, false, false, false));
        NamedCommands.registerCommand("Fire", new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.runLoaderVoltage)));
        NamedCommands.registerCommand("Check Note", new InstantCommand(() -> s_Shooter.checkNote()));
        NamedCommands.registerCommand("Got Note", new ConditionalCommand(
            new WaitCommand (15),
            new InstantCommand (),
            () -> s_Shooter.gotNote
        ));
        NamedCommands.registerCommand("Not Got Note", new ConditionalCommand(
            new InstantCommand (),
            new WaitCommand (15),
            () -> s_Shooter.gotNote
        ));

        //TODO Shoot on the Move in Auto?
        
        autoChooser = AutoBuilder.buildAutoChooser();

        
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("driver/Auto Chooser", autoChooser);
        
    }    
    
    private boolean prepareShot() {

        return (s_ShooterPivot.atPosition() == true && s_Shooter.isUpToSpeed() == true);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        
        // zero gyro
        driverY.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // aim speaker
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            driverLeftTrigger.whileTrue(new TeleopSwerve(
                    s_Swerve, 
                    () -> -driver.getRawAxis(leftY), 
                    () -> -driver.getRawAxis(leftX), 
                    () -> 0, //driver.getRawAxis(rightX),
                    () -> false,
                    () -> s_Eyes.getMovingTargetRotation(),
                    () -> true,
                    rotationSpeed,
                    true
                ).alongWith(new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter, false, true, false))

            );
        } else {
            driverLeftTrigger.whileTrue(new TeleopSwerve(
                    s_Swerve, 
                    () -> driver.getRawAxis(leftY), 
                    () -> driver.getRawAxis(leftX), 
                    () -> 0,//driver.getRawAxis(rightX),
                    () -> false,                    
                    () -> s_Eyes.getMovingTargetRotation(),
                    () -> true,
                    rotationSpeed,
                    true
                ).alongWith(new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter, false ,true, false))

            );
        }
        

        // aim speaker with elevator
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            driverB.whileTrue(
                new ParallelCommandGroup(
                    new TeleopSwerve(
                            s_Swerve, 
                            () -> -driver.getRawAxis(leftY), 
                            () -> -driver.getRawAxis(leftX), 
                            () -> driver.getRawAxis(rightX),
                            () -> false,
                            () -> s_Eyes.getTargetRotation(),
                            () -> false,//driverB.getAsBoolean(),
                            rotationSpeed,
                            false //TODO Determine if we want auto rotation aiming
                        ).alongWith(new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter, true, false, false)),
                    new InstantCommand(() -> s_Elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL))
                )
            ).onFalse(
                new ParallelCommandGroup(
                    new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)),
                    new InstantCommand(() -> s_Elevator.SetElevatorPosition(0))
                )
            );
        } else {
            driverB.whileTrue(
                new ParallelCommandGroup(
                    new TeleopSwerve(
                            s_Swerve, 
                            () -> driver.getRawAxis(leftY), 
                            () -> driver.getRawAxis(leftX), 
                            () -> driver.getRawAxis(rightX),
                            () -> false,
                            () -> s_Eyes.getTargetRotation(), 
                            () -> false,//driverB.getAsBoolean(),
                            rotationSpeed,
                            false //TODO: Determine if we want auto rotation aiming
                        ).alongWith(new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter, true, false, false)),
                    new InstantCommand(() -> s_Elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL))
                )
            ).onFalse(
                new ParallelCommandGroup(
                    new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)),
                    new InstantCommand(() -> s_Elevator.SetElevatorPosition(0))
                )
            );
        }

        // shoot speaker
        driverRightTrigger.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.runLoaderVoltage))
            )
        ).onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(0))
            )
        );

        

        // intake
        driverX.whileTrue(new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.setIntakePivotPosition(s_Intake.intakeGroundPosition)),
                new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotIntakePosition)),
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.runLoaderVoltage))
                ),
            s_Intake.IntakeAtPosition().withTimeout(0.25), //TODO figure out if this actually is working or just time delay
            new InstantCommand(() -> s_Intake.setIntakeVoltage(s_Intake.runIntakeVoltage)).repeatedly()) 
            .until(() -> !s_Shooter.getBreakBeamOutput())
            .andThen(new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.setIntakePivotPosition(s_Intake.intakeSafePosition)),
                new InstantCommand(() -> s_Intake.setIntakeVoltage(s_Intake.stopIntakeVoltage)),
                new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)),
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.stopLoaderVoltage)),
                new InstantCommand(() -> s_Eyes.limelight.setLEDMode_ForceBlink("")),
                new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1))
            )))
            .onFalse(new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.setIntakePivotPosition(s_Intake.intakeSafePosition)),
                new InstantCommand(() -> s_Intake.setIntakeVoltage(s_Intake.stopIntakeVoltage)),
                new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)),
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.stopLoaderVoltage)),
                new InstantCommand(() -> s_Eyes.limelight.setLEDMode_ForceOff("")),
                new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0))));

        // outake
        driverA.whileTrue(new SequentialCommandGroup(
            new InstantCommand(() -> s_Intake.setIntakePivotPosition(s_Intake.intakeGroundPosition)),
            new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotIntakePosition)),
            new WaitCommand(0.25),
            new InstantCommand(() -> s_Intake.setIntakeVoltage(s_Intake.reverseIntakeVoltage)),
            new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.reverseLoaderVoltage))
        )).onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.setIntakePivotPosition(s_Intake.intakeSafePosition)),
                new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)),
                new InstantCommand(() -> s_Intake.setIntakeVoltage(s_Intake.stopIntakeVoltage)),
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.stopLoaderVoltage))
            )
        );

        

        //Climbing
        driverRB.onTrue(new ConditionalCommand(
            //climb pull up
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.setShooterVoltage(0,0)),
                new InstantCommand(() -> s_Elevator.SetElevatorPosition(2.0)),
                new InstantCommand(() -> SmartDashboard.putString("ClimbCommand", "up")),
                new InstantCommand(() -> s_Elevator.isClimbed(true))),

            //Reach for Climb             
            new SequentialCommandGroup(
                new InstantCommand(() -> s_Elevator.resetElevatorReverseSoftlimit()),
                new InstantCommand(() -> s_Shooter.setShooterVoltage(0,0)),
                new InstantCommand(() -> s_Elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL)),
                s_Elevator.ElevatorAtPosition(Constants.ELEVATOR_SAFE_LEVEL),
                new ParallelCommandGroup(
                    new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotClimbPosition)),
                    new InstantCommand(() -> s_Elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL)),
                    new InstantCommand(() -> s_Elevator.isClimbed(false))
            )), 
            () -> {
                if(s_Elevator.isClimbed){
                return false;
            } else{
                return true;
            }
        }));
        //climb down more
        driverDpadDown.onTrue(
            new InstantCommand(() -> s_Elevator.SetElevatorPosition(0.0)));

        // abort climb
        driverSelect.onTrue(

            new SequentialCommandGroup(
                new InstantCommand(() -> s_Shooter.setShooterVoltage(0,0)),
                new InstantCommand(() -> s_Elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL)),
                new InstantCommand(() -> SmartDashboard.putString("ClimbCommand", "down")),
                s_Elevator.ElevatorAtPosition(),
                new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)),
                new InstantCommand(() -> SmartDashboard.putString("ShooterPivot", "return")),
                s_ShooterPivot.ShooterPivotAtPosition(),
                new InstantCommand(() -> s_Elevator.SetElevatorPosition(0)),
                new InstantCommand(() -> s_Elevator.isClimbed(true))
            )
        );

        

        // generate and run path to closest trap
        driverStart.whileTrue(new ParallelCommandGroup(new ConditionalCommand(new InstantCommand(() -> {
                s_Swerve.onTheFly(() -> s_Eyes.trapPath).schedule();

            }),
            new InstantCommand(),

            () -> s_Eyes.closeToTrap),

            new Blow(s_Blower)
            //     .until(() -> s_Eyes.atTrap())
            //     .andThen(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1)))) //TODO Test this, was only running on init earlier, may need to be run command
        )
        ).onFalse(s_Swerve.getDefaultCommand()); //TODO let driver know we are in position to trap via rumble
        

        //Feed
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            driverRStick.whileTrue(new TeleopSwerve(
                    s_Swerve, 
                    () -> -driver.getRawAxis(leftY), 
                    () -> -driver.getRawAxis(leftX), 
                    () -> 0,
                    () -> false,
                    () -> s_Eyes.getFeedRotation(),
                    () -> false,
                    rotationSpeed,
                    true
                ).alongWith(new Shooting(s_ShooterPivot,s_Shooter, 65.0, 117.0))//new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter, false, false, true))
            ).onFalse(new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)));
        } else {
            driverRStick.whileTrue(new TeleopSwerve(
                    s_Swerve, 
                    () -> driver.getRawAxis(leftY), 
                    () -> driver.getRawAxis(leftX), 
                    () -> 0,
                    () -> false,
                    () -> s_Eyes.getFeedRotation(),
                    () -> false,
                    rotationSpeed,
                    true
                ).alongWith(new Shooting(s_ShooterPivot,s_Shooter, 65.0, 117.0)))
                .onFalse(new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition))


            );
        }

        //reach amp 
        driverLB.whileTrue(
            new ParallelCommandGroup(
                new PrepareFeed(s_Shooter, 25),
                new SequentialCommandGroup(
                    new InstantCommand(() -> s_Elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL)),
                    s_Elevator.ElevatorAtPosition(Constants.ELEVATOR_SAFE_LEVEL),
                    new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotAmpPosition))
                )
            )
        ).onFalse(
                new SequentialCommandGroup(

                    new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)),
                    s_ShooterPivot.ShooterPivotAtPosition(),
                new InstantCommand(() -> s_Elevator.SetElevatorPosition(0.0)),
                    s_Elevator.ElevatorAtPosition(0.0),
                new WaitCommand(0.5),
                new InstantCommand(() -> s_Elevator.resetEncoder())
                )
        );

        //source intake
                    operatorY.whileTrue(
                    new SequentialCommandGroup(
                        new InstantCommand(() -> s_Shooter.setShooterVoltage(0,0)),
                        new InstantCommand(() -> s_Elevator.SetElevatorPosition(8.85)),
                        s_Elevator.ElevatorAtPosition(),
                        
                        new ParallelCommandGroup(
                            new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(325)),
                            new RunLoader(s_Shooter).until(() -> !s_Shooter.getBreakBeamOutput())
                    .andThen(new ParallelCommandGroup(
                        new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1))
                    )))) 
                        
                ).onFalse(
                    new ParallelCommandGroup(
                        new ParallelCommandGroup(
                            new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0))
                        ),
                        new SequentialCommandGroup(
                            new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)),
                            s_ShooterPivot.ShooterPivotAtPosition(),
                            new InstantCommand(() -> s_Elevator.SetElevatorPosition(0)),
                            s_Elevator.ElevatorAtPosition(0.0),
                            new WaitCommand(0.5),
                            new InstantCommand(() -> s_Elevator.resetEncoder())
                        )
                    )
                );
    
        operatorRightTrigger.onTrue(
             new ParallelCommandGroup(
                 new InstantCommand(() -> s_Shooter.setLoaderVoltage(6)),
                 new InstantCommand(() -> s_Shooter.setShooterVoltage(6, -6))
             )
         ).onFalse(
             new ParallelCommandGroup(
                 new InstantCommand(() -> s_Shooter.setLoaderVoltage(0)),
                 new InstantCommand(() -> s_Shooter.setShooterVoltage(0, 0))
             )
         );
        

        // dummy shoot commands
        operatorDpadDown.whileTrue(new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter, 1.25))
            .onFalse(new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)));
        operatorDpadLeft.whileTrue(new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter, 2.4))
            .onFalse(new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)));
        operatorDpadUp.whileTrue(new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter, 3.17))
            .onFalse(new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)));
        //prepare feed
        operatorA.whileTrue(new PrepareFeed(s_Shooter, 65.0));
        // prepare trap
        operatorX.whileTrue(new Shooting(s_ShooterPivot, s_Shooter, 45.0, s_ShooterPivot.shooterPivotStowPosition));
        //tuning shots
        //operatorB.whileTrue(new ManualShooting(s_Shooter, s_ShooterPivot));
        

    }

    public void rumbleControllers() {
        if (s_Eyes.controllerRumble == true) {
            driver.setRumble(RumbleType.kBothRumble, 1);
        } else {
            driver.setRumble(RumbleType.kBothRumble, 0);
        }
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return autoChooser.getSelected();
        
    }


}
