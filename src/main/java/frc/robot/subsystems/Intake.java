/*
 * This subsystem controls the robot intake. It includes
 * both the intake pivot motor and the intake rollers.
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // local constants

  // IDs
  private final int intakeMotorID = 10;
  private final int intakePivotID = 11;
  private final int intakePivotEncoderID = 9;

  // positions
  // NOTE: these positions are also used in robotcontainer.
  public final double intakeSafePosition = -32.95;
  public final double intakeGroundPosition = -130.25; //-142
  public final double intakeSourcePosition = intakeSafePosition;

  // PID values
  private final double intakePValue = 0.2;
  private final double intakeIValue = 0.0;
  private final double intakeDValue = 0.0;

  private final double intakePivotMotorGearRatio = 100.0;

  private final double magnetOffSet = 0.0;

  // Kermit Limits
  private final int intakeCurrentLimit = 80;
  private final int intakePivotCurrentLimit = 60;

  // local variables
  public double runIntakeVoltage = -12.0;
  public double reverseIntakeVoltage = 12.0;
  public double stopIntakeVoltage = 0.0;

  private double m_setPoint;
  private double intakePivotVoltage;

  // WPILib class objects
  private CANSparkMax m_Intake;
  private CANSparkMax m_IntakePivot;
  private CANcoder e_intakePivot;
  private RelativeEncoder e_intakePivotIntegrated;
  private PIDController pid;


  // constructor
  public Intake() {

    // instantiate objects
    m_Intake = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
    m_IntakePivot = new CANSparkMax(intakePivotID, MotorType.kBrushless);
    e_intakePivot = new CANcoder(intakePivotEncoderID);

    // configure cancoder
    CANcoderConfigurator configuator = e_intakePivot.getConfigurator();
    configuator.apply(
      new MagnetSensorConfigs()
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withMagnetOffset(magnetOffSet)
    );    
    
    m_Intake.setSmartCurrentLimit(intakeCurrentLimit);
    m_IntakePivot.setSmartCurrentLimit(intakePivotCurrentLimit);

    // TODO: Go over this part with Dylan and student that wrote this. Can we simplify this?
    e_intakePivotIntegrated = m_IntakePivot.getEncoder();
    e_intakePivotIntegrated.setPositionConversionFactor(360 / intakePivotMotorGearRatio);
    e_intakePivotIntegrated.setPosition(e_intakePivot.getPosition().getValue() * 360);
    e_intakePivotIntegrated.setVelocityConversionFactor(360 / intakePivotMotorGearRatio);

    // create PID loop for intake pivot
    pid = new PIDController(intakePValue, intakeIValue, intakeDValue);

    // go to intake safe position on initialization
    m_setPoint = intakeSafePosition;

    // stop intake on initialization
    setIntakeVoltage(stopIntakeVoltage);

  }

  /*
   * This method will set the position of the shooter pivot.
   * 
   * parameters:
   * setpoint       (double)
   * 
   * returns:
   * none
   */
  public void setIntakePivotPosition(double setpoint) {
    m_setPoint = setpoint;
  }

  /*
   * This method will run the intake motor at a desired voltage.
   * It can be used to start and stop the intake.
   * 
   * parameters:
   * voltage         (double)
   * 
   * returns:
   * none
   */
  public void setIntakeVoltage(double intakeSpeedVoltage) {
    m_Intake.setVoltage(intakeSpeedVoltage);
    }

  /*
   * This method will get the position of the intake
   * cancoder in degrees.
   * 
   * parameters:
   * none
   * 
   * returns:
   * cancoder degrees     (double)
   */
  public double cancoderInDegrees() {
    return e_intakePivot.getPosition().getValue() * 360;
  }
  
  double getTargetIntakePosition() {
    return intakeGroundPosition;
  }


  public boolean atPosition() {

        double error = Math.abs(e_intakePivot.getPosition().getValueAsDouble() - getTargetIntakePosition());

        if (5.0 >= error) {
            return true;

        } else {
            return false;
        }

        }

        public Command IntakeAtPosition(){
          return Commands.waitUntil(() -> atPosition());
  }

   @Override
   public void periodic() {
  
    // move the intake pivot motor to the current desired position
    intakePivotVoltage = pid.calculate(e_intakePivotIntegrated.getPosition(), m_setPoint)/*+ feedForward) */;
    m_IntakePivot.setVoltage(intakePivotVoltage);

    // log values
    SmartDashboard.putNumber("Intake Voltage", intakePivotVoltage);
    SmartDashboard.putNumber("Intake CANcoder", cancoderInDegrees());
    SmartDashboard.putNumber("Intake Pivot Motor Position", e_intakePivotIntegrated.getPosition());
    SmartDashboard.putNumber("Intake setpoint", m_setPoint);
    SmartDashboard.putNumber("Intake Current", m_Intake.getOutputCurrent());
    SmartDashboard.putNumber("debug/Intake Voltage", intakePivotVoltage);
    SmartDashboard.putNumber("debug/Intake CANcoder", cancoderInDegrees());
    SmartDashboard.putNumber("debug/Intake Pivot Motor Position", e_intakePivotIntegrated.getPosition());
    SmartDashboard.putNumber("debug/Intake setpoint", m_setPoint);
    SmartDashboard.putNumber("debug/Intake Current", m_Intake.getOutputCurrent());
   }
}
