/*
 * This method controls the shooting mechanism. It
 * manages both the shooter and the loader.
 * 
 */
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase {

  // local constants
  
  //IDs
  private final int leftShooterMotorID = 13;
  private final int rightShooterMotorID = 14;
  private final int loaderMotorID = 16;
  private final int breakBeamID = 2;//0
  

  // loader speeds
  public final double runLoaderVoltage = 12.0;
  public final double reverseLoaderVoltage = -3.0;
  public final double stopLoaderVoltage = 0.0;

  // shooter speeds
  public final double runShooterVoltage = 6.0;
  public final double reverseShooterVoltage = 3.0;
  public final double stopShooterVoltage = 0.0;
  public final double shooterSpeedToleranceRPS = 100.0/60.0; //100 RPM

  public final double shotSpeedRPS = 90;
  public final double shooterSpinReduction = 1.0;
  private final int currentLimit = 80;


  // left shooter motor PID
  private final double lShooterMotorPGains = 0.2;
  private final double lShooterMotorIGains = 0.0;
  private final double lShooterMotorDGains = 0.0;
  private final double lShooterMotorSGains = 0.0;
  private final double lShooterMotorVGains = 0.12;

  // right shooter motor PID
  private final double rShooterMotorPGains = 0.2;
  private final double rShooterMotorIGains = 0.0;
  private final double rShooterMotorDGains = 0.0;
  private final int m_LoaderCurrentLimit = 40;
  
  private final double rShooterMotorSGains = 0.0;
  private final double rShooterMotorVGains = 0.12;

  // WPILib class objects
  public TalonFX m_leftShooter;
  public TalonFX m_rightShooter;
  private TalonFX m_loader;

  private Slot0Configs slotConfigsR;
  private Slot0Configs slotConfigsL;

  private VelocityVoltage rm_request;
  private VelocityVoltage lm_request;

  public DigitalInput breakBeam;
  
  private TalonFXConfigurator configF;
  public double m_setSpeed = 0.0;

  public boolean gotNote = false;

  // constructor
  public Shooter() {
    // motors
    m_leftShooter = new TalonFX(leftShooterMotorID);
    m_rightShooter = new TalonFX(rightShooterMotorID);
    m_loader = new TalonFX(loaderMotorID);

    // break beam sensor
    breakBeam = new DigitalInput(breakBeamID);

    configF = m_loader.getConfigurator();

    configF.apply(
      new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(m_LoaderCurrentLimit)
    );

    // right shooter motor configuration
    slotConfigsR = new Slot0Configs();
    slotConfigsR.kS = rShooterMotorSGains;
    slotConfigsR.kV = rShooterMotorVGains;
    slotConfigsR.kP = rShooterMotorPGains; 
    slotConfigsR.kI = rShooterMotorIGains;
    slotConfigsR.kD = rShooterMotorDGains;

    // left shooter motor configuration
    slotConfigsL = new Slot0Configs();
    slotConfigsL.kS = lShooterMotorSGains;
    slotConfigsL.kV = lShooterMotorVGains;
    slotConfigsL.kP = lShooterMotorPGains;
    slotConfigsL.kI = lShooterMotorIGains;
    slotConfigsL.kD = lShooterMotorDGains;

    // set shooter motors to coast mode
    m_leftShooter.setNeutralMode(NeutralModeValue.Coast);
    m_rightShooter.setNeutralMode(NeutralModeValue.Coast);

    // log data
    SmartDashboard.putNumber("RShooter kP", slotConfigsR.kP);
    SmartDashboard.putNumber("RShooter kV", slotConfigsR.kV);
    SmartDashboard.putNumber("RShooter Speed", 0);
    SmartDashboard.putNumber("LShooter kP", slotConfigsL.kP);
    SmartDashboard.putNumber("LShooter kV", slotConfigsL.kV);
    SmartDashboard.putNumber("LShooter Speed", 0);

    // configure shooter motors
    shootingMotorsConfig();

    // PID Velocity target
    rm_request = new VelocityVoltage(0);
    lm_request = new VelocityVoltage(0);
  }

  /*
   * This method will set the voltage of the loader.
   * It can be used to start and stop the loader.
   * 
   * parameters:
   * desired voltage      (double)
   * 
   * returns:
   * none
   */
  public void setLoaderVoltage(double loaderSpeedVoltage) {
    m_loader.setVoltage(loaderSpeedVoltage);  
  }

  /*
   * This method will get the output of the break beam sensor
   * as a boolean
   * 
   * parameters:
   * none
   * 
   * returns:
   * none
   */
  public boolean getBreakBeamOutput() {
    return breakBeam.get();

  }

  public void checkNote() {
    gotNote = !getBreakBeamOutput();
  }

  public Trigger getBreakBeamTrigger() {
    return new Trigger(() -> breakBeam.get());
  }

  /*
   * This method will configure the left and right shooting motors.
   * 
   * parameters:
   * none
   * 
   * returns:
   * none
   */
  public void shootingMotorsConfig() {

    // get configurations
    slotConfigsR.kP = SmartDashboard.getNumber("RShooter kP", 0);
    slotConfigsR.kV = SmartDashboard.getNumber("RShooter kV", 0);
    slotConfigsL.kP = SmartDashboard.getNumber("LShooter kP", 0);
    slotConfigsL.kV = SmartDashboard.getNumber("LShooter kV", 0);



    // set configurations to motors
    m_rightShooter.getConfigurator().apply(slotConfigsR);
    m_leftShooter.getConfigurator().apply(slotConfigsL);

    m_rightShooter.getConfigurator().apply(new CurrentLimitsConfigs()
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(currentLimit));

    m_leftShooter.getConfigurator().apply(new CurrentLimitsConfigs()
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(currentLimit));
  }

  /*
   * This method will set the speed of the shooter motors using a PID loop
   * with a velocity input.
   * 
   * parameters:
   * right shooter motor velocity       (double)
   * left shooter motor velocity        (double)
   * 
   * returns:
   * none
   */
  public void shootingMotorsSetControl(double rightShooterSpeed, double leftShooterSpeed) {
    m_setSpeed = rightShooterSpeed;
    m_rightShooter.setControl(rm_request.withVelocity(rightShooterSpeed));
    m_leftShooter.setControl(lm_request.withVelocity(-leftShooterSpeed));

  }

  /*
   * This method will set the speed of the shooter by assigning a voltage to
   * the motors.
   * 
   * parameters:
   * right shooter motor voltage         (double)
   * left shooter motor voltage          (double)
   * 
   * returns:
   * none
   */
  public void setShooterVoltage(double rightVoltage, double leftVoltage) {

    m_rightShooter.setVoltage(rightVoltage);
    m_leftShooter.setVoltage(leftVoltage);

  }


   public boolean isUpToSpeed() {

    
     double actualSpeed = m_leftShooter.getVelocity().getValueAsDouble();
     double error = Math.abs(actualSpeed + m_setSpeed);

     SmartDashboard.putNumber("shooterActualSpeed", actualSpeed);
     SmartDashboard.putNumber("shooterSetSpeed", m_setSpeed);

     if (error <= shooterSpeedToleranceRPS) {
       return true;
     } else {
       return false;
     }
   }

  @Override
  public void periodic() {

    // log shooting data
    SmartDashboard.putNumber("Right Shooter Speed", m_rightShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Shooter Speed", m_leftShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Shooter Voltage", m_rightShooter.getMotorVoltage().getValue());
    SmartDashboard.putNumber("Left Shooter Voltage", m_leftShooter.getMotorVoltage().getValue());
    SmartDashboard.putNumber("Right Shooter Temp", m_rightShooter.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Left Shooter Temp", m_leftShooter.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putBoolean("Break Beam Sensor", getBreakBeamOutput());
    SmartDashboard.putBoolean("driver/Break Beam Sensor", getBreakBeamOutput());
    SmartDashboard.putNumber("Right Shooter Current", m_rightShooter.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Left Shooter Current", m_leftShooter.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("debug/Right Shooter Speed", m_rightShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("debug/Left Shooter Speed", m_leftShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("debug/Right Shooter Temp", m_rightShooter.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("debug/Left Shooter Temp", m_leftShooter.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putBoolean("debug/Break Beam Sensor", getBreakBeamOutput());
    SmartDashboard.putNumber("debug/Right Shooter Current", m_rightShooter.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("debug/Left Shooter Current", m_leftShooter.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Right Shooter Stator Current", m_rightShooter.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Left Shooter Stator Current", m_leftShooter.getStatorCurrent().getValueAsDouble());

  }
}
