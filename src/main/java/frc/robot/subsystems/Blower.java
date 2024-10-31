// package frc.robot.subsystems;


// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// public class Blower extends SubsystemBase {
//   /* Instance Variables
//    * This section should contain any variables that need to 
//    * be accessed by your entire subsystem/class. This usually includes any phyical motors or sensors that are a part of your subsystem,
//    * as well as any data that needs to be accessed by your 
//    * entire subsystem/class E.G. the height of the elevator 
//    * or angle of the wrist.
//    */

//   /* These are variable declarations. Their access modifiers,
//    * types, and names are specified but they are not given a 
//    * value yet. They should be given a value in the 
//    * constructor. In this example, they is private, which 
//    * means that nothing outside of this class can access it.
//    */
//   private CANSparkMax motor;
//   private double internalData;
//   private int blowerID = 19;
  
//   /* Constructor
//    * The Constructor is a special type of method that gets 
//    * called when this subsystem/class is created, usually 
//    * when the robot turns on. You should give a value to most
//    * instance variables created above in this method. You can
//    * also do anything else that you want to happen right away.
//    */
//   public Blower() {
//     /* These are variable initializations, where a variable 
//      * is given a value. In this case, the `new` keyword is 
//      * used to create a new CANSparkMax object and give it 
//      * to `motor` as it's value. 
//      */
//     motor = new CANSparkMax(blowerID, MotorType.kBrushed);
//   }

//   public void runBlower() {
//     motor.setVoltage(12);
//   }

//   public void stopBlower() {
//     motor.setVoltage(0);
//   }

//   /* The below method is included in every Subsystem. You can
//    * think of it as an infinite loop that runs constantly 
//    * while the robot is on.
//    * 
//    */
//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }