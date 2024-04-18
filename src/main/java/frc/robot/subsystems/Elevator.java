// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

public class Elevator extends SubsystemBase {
  /* Instance Variables
   * This section should contain any variables that need to 
   * be accessed by your entire subsystem/class. This usually includes any phyical motors or sensors that are a part of your subsystem,
   * as well as any data that needs to be accessed by your 
   * entire subsystem/class E.G. the height of the elevator 
   * or angle of the wrist.
   */

  private static final ProfiledPIDController ff = null;
  /* These are variable declarations. Their access modifiers,
   * types, and names are specified but they are not given a 
   * value yet. They should be given a value in the 
   * constructor. In this example, they is private, which 
   * means that nothing outside of this class can access it.
   */
  private CANSparkMax m_elevator1;
  private CANSparkMax m_elevator2;
  private SparkPIDController pidController; 
  private RelativeEncoder e_Elevator;
  private ProfiledPIDController controller;

  private int elevatorCurrentLimit = 60;

  public double targetElevatorPosition = 0;

  private double heightlimit = 16;
  public double elevatorspeed = 0.1;
  public double restingposition = 0;
  public double climbingPosition = -2.0;
  public double shootingPosition = 12.0;

  private double elevatorP = 1.5; //4.0
  private double elevatorI = 0.0;
  private double elevatorD = 0.0;

  private double voltage = 0.0;
  public boolean isClimbed = true;
 

  /* Constructor
   * The Constructor is a special type of method that gets 
   * called when this subsystem/class is created, usually 
   * when the robot turns on. You should give a value to most
   * instance variables created above in this method. You can
   * also do anything else that you want to happen right away.
   */
  public Elevator() {
    /* These are variable initializations, where a variable 
     * is given a value. In this case, the `new` keyword is 
     * used to create a new CANSparkMax object and give it 
     * to `motor` as it's value. 
     */
    m_elevator1 = new CANSparkMax(17, MotorType.kBrushless);
    m_elevator2 = new CANSparkMax(18, MotorType.kBrushless);

    controller = new ProfiledPIDController(elevatorP, elevatorI, elevatorD, new Constraints(80, 1000));
    controller.setTolerance(100, 100);

    e_Elevator = m_elevator1.getEncoder();

    m_elevator1.restoreFactoryDefaults();
    m_elevator1.setInverted(false);

    pidController = m_elevator1.getPIDController();
    
    m_elevator1.setSmartCurrentLimit(elevatorCurrentLimit);
    m_elevator2.setSmartCurrentLimit(elevatorCurrentLimit);

    m_elevator1.setSoftLimit(SoftLimitDirection.kForward, (float)inchesToMotorRotations(heightlimit));
    m_elevator1.setSoftLimit(SoftLimitDirection.kReverse,(float)inchesToMotorRotations(restingposition));
    
    m_elevator2.restoreFactoryDefaults();
    m_elevator2.follow(m_elevator1, true);

    pidController = m_elevator2.getPIDController();
    m_elevator2.setSoftLimit(SoftLimitDirection.kForward, (float)inchesToMotorRotations(heightlimit));
    m_elevator2.setSoftLimit(SoftLimitDirection.kReverse,(float)inchesToMotorRotations(restingposition));

    m_elevator1.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_elevator1.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_elevator2.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_elevator2.enableSoftLimit(SoftLimitDirection.kReverse, true);


    m_elevator1.setIdleMode(IdleMode.kBrake);
    m_elevator2.setIdleMode(IdleMode.kBrake);

    e_Elevator.setPositionConversionFactor(Constants.ELEVATOR_ROTATIONS_TO_IN);
    e_Elevator.setVelocityConversionFactor(Constants.ELEVATOR_ROTATIONS_TO_IN);

    double kP = 0.1; 
    double kI = 1e-4;
    double kD = 1; 
    double kIz = 0; 
    double kFF = 0; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  /*This method tells the robot how many rotations are needed for moving the elevator. SetReference is the 
  number of, in this case, rotations needed to be executed by the robot
  */
   
  public void SetReference(double rotations){
  
    pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  /* Methods
   * The below section should contain any methods you want 
   * to use in your class.
   */

   /* This method makes the elevator go up until it hits the soft limit*/
  public void move(double speed) {
    m_elevator1.set(speed);
  }


  /* Sets the Target Elevator Position in inches.*/
  public double getTargetElevatorPosition(){
      return targetElevatorPosition;
  }

  public void climb() {
  m_elevator1.setSoftLimit(SoftLimitDirection.kReverse,(float)inchesToMotorRotations(climbingPosition));  
  m_elevator2.setSoftLimit(SoftLimitDirection.kReverse,(float)inchesToMotorRotations(climbingPosition));
  SetElevatorPosition(climbingPosition);
  }

  public void resetElevatorReverseSoftlimit(){
  m_elevator1.setSoftLimit(SoftLimitDirection.kReverse,(float)inchesToMotorRotations(restingposition));  
  m_elevator2.setSoftLimit(SoftLimitDirection.kReverse,(float)inchesToMotorRotations(restingposition));
  }

    private double motorRotationsToInches(double rotations) {
        return rotations * Constants.ELEVATOR_ROTATIONS_TO_IN;
    }

    private double inchesToMotorRotations(double inches) {
        return inches / Constants.ELEVATOR_ROTATIONS_TO_IN;
    }

    public boolean atPosition() {

        double error = Math.abs(e_Elevator.getPosition() - getTargetElevatorPosition());

        if (Constants.ELEVATOR_TOLERANCE >= error) {
            return true;

        } else {
            return false;
        }

        }

        public boolean atPosition(double setPosition) {

        double error = Math.abs(e_Elevator.getPosition() - setPosition);

        if (Constants.ELEVATOR_TOLERANCE >= error) {
            return true;

        } else {
            return false;
        }

        }

  /* The below method is public, which means that it can be 
   * accessed outside of this class. It also takes in a 
   * single double value as a parameter. It then takes that 
   * value (called `number`) and multiplies it by 
   * `internalData` to change it's value.
   */
  public void stop() {
    m_elevator1.set(0);
  }
/*This method makes the elevator go down until */
public void down() {
  m_elevator1.set(elevatorspeed);
}
/*This method sets the elevator to a shooting position */
private void goToShootingPos(){
if (m_elevator1.getEncoder().getPosition() == shootingPosition){
  stop();
}else if(m_elevator1.getEncoder().getPosition() < shootingPosition){
  move(elevatorspeed);
}else{
  down();
}
}

public void isClimbed(boolean climbState) {
  isClimbed = climbState;
}


public void resetEncoder() {
  e_Elevator.setPosition(0.0);
}


  /* The below method is included in every Subsystem. You can
   * think of it as an infinite loop that runs constantly 
   * while the robot is on.
   * 
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isEnabled()){

            // This method will be called once per scheduler run
            // TODO: Test that .getPosition() gives us the elevator position in inches
            voltage = controller.calculate(e_Elevator.getPosition(), targetElevatorPosition);
            //double feedforward = ff.calculate(/*double*/ e_Elevator.getVelocity());
            MathUtil.clamp(voltage, -12, 12);

            m_elevator1.setVoltage(voltage);
            m_elevator2.setVoltage(voltage);

            
        }
        //logData();
        SmartDashboard.putNumber("ELEVATOR TARGET POSITION", targetElevatorPosition);
        SmartDashboard.putNumber("Elevator Encoder Value: ", getPosition());
        SmartDashboard.putNumber("Current Elevator Position",e_Elevator.getPosition());
        SmartDashboard.putNumber("Elevator Voltage", voltage);
        SmartDashboard.putNumber("debug/ELEVATOR TARGET POSITION", targetElevatorPosition);
        SmartDashboard.putNumber("debug/Elevator Encoder Value: ", getPosition());
        SmartDashboard.putNumber("debug/Current Elevator Position",e_Elevator.getPosition());
        SmartDashboard.putNumber("debug/Elevator Voltage", voltage);
        
       // logData();
      
  }
   
private double getPosition() {
    // TODO Auto-generated method stub
    return e_Elevator.getPosition();
  }

  public void SetElevatorPosition (double inches){
          targetElevatorPosition = inches;
  }

      public Command ElevatorAtPosition(){
          return Commands.waitUntil(() -> atPosition());
  }

      public Command ElevatorAtPosition(double setPosition){
        return Commands.waitUntil(() -> atPosition(setPosition));
  }
         
}

            