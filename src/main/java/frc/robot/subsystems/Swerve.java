/*
 * This subsystem manages the 
 */

package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.lib.util.FieldRelativeAccel;
import frc.lib.util.FieldRelativeAccel;
import frc.lib.util.FieldRelativeSpeed;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {

    // subsystems

    // WPILib class objects
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDriveKinematics swerveKinematics = Constants.Swerve.swerveKinematics;
    public Translation2d frontLeftModule;
    public Translation2d frontRightModule;
    public Translation2d backLeftModule;
    public Translation2d backRightModule;
    public SwerveModuleState[] swerveModuleStates;
    private Field2d m_field;
    private DoubleArrayPublisher moduleStatePublisher;
    private StructPublisher<Pose2d> posePublisher;
    public StructArrayPublisher<SwerveModuleState> swerveKinematicsPublisher;
    public StructPublisher<Pose2d> estimatedRobotPosePublisher;
    public SwerveDrivePoseEstimator m_poseEstimator;
    public FieldRelativeSpeed fieldRelativeVelocity;
    public FieldRelativeSpeed lastFieldRelativeVelocity;
    public FieldRelativeAccel fieldRelativeAccel;
    private double time;
    private double lastTime = 0.0;
    

    // constructor
    public Swerve() {

        // instantiate objects 
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        m_field = new Field2d();
        fieldRelativeVelocity = new FieldRelativeSpeed();
        lastFieldRelativeVelocity = new FieldRelativeSpeed();
        fieldRelativeAccel = new FieldRelativeAccel();

        // set gyro
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(Constants.Swerve.gyroOffset);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // delay reseting modules utill robRIO finishes startup
        Timer.delay(1.0);
        resetModulesToAbsolute();
        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, getGyroYaw(), getModulePositions());
      
        // create loggers
        moduleStatePublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("/ModuleStates").publish();
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("/MyPose", Pose2d.struct).publish();
        swerveKinematicsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveModuleStates", SwerveModuleState.struct).publish();
        estimatedRobotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/EstimatedRobotPose", Pose2d.struct).publish();
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();

        // create autobuilder
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier use getPose to disable apriltags for auto
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        Constants.Swerve.maxSpeed, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
      
                // create pose estimator
        m_poseEstimator =
                new SwerveDrivePoseEstimator(
                   Constants.Swerve.swerveKinematics,
                   gyro.getRotation2d(),
                   getModulePositions(),
                   new Pose2d(),
                   VecBuilder.fill(0.1, 0.1, 0.1),
                   VecBuilder.fill(1.5, 1.5, 1.5)
                );

    }


    /*
     * This method will drive the swerve drive using translation and rotation vectors
     * 
     * Parameters:
     * translation X           (double)
     * translation Y           (double)
     * Rotation                (double)
     * is field relative       (boolean)
     * is open loop            (boolean)
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    } 
    
    /*
     * This method will get the modules states and convert
     * them into a ChassisSpeeds object
     * 
     * parameters:
     * none
     * 
     * returns:
     * swerve ChassiSpeeds object
     */
    public ChassisSpeeds getChassisSpeed() {

        return swerveKinematics.toChassisSpeeds(getModuleStates());

    }

    /*
     * This method will set the swerve modules to match a
     * ChassisSpeed parameter.
     * 
     * parameters:
     * ChassisSpeeds input         (ChassisSpeeds)
     * 
     * returns:
     * none
     */
    public void setChassisSpeed(ChassisSpeeds chassisSpeed) {

        SwerveModuleState[] desiredStates = swerveKinematics.toSwerveModuleStates(chassisSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        moduleStatePublisher.set(new double[] {
            desiredStates[0].angle.getDegrees(), desiredStates[0].speedMetersPerSecond,
            desiredStates[1].angle.getDegrees(), desiredStates[1].speedMetersPerSecond,
            desiredStates[2].angle.getDegrees(), desiredStates[2].speedMetersPerSecond,
            desiredStates[3].angle.getDegrees(), desiredStates[3].speedMetersPerSecond,
        });

        setModuleStates(desiredStates);
        
    }
    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }


    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        } else {
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d(Math.PI)));
        }
        
    }

    public Rotation2d getGyroYaw(){
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Command onTheFly(PathPlannerPath path) {
    
        return new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        Constants.Swerve.maxSpeed, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                this // Reference to this subsystem to set requirements
        );
      }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        time = Timer.getFPGATimestamp();
        fieldRelativeVelocity = new FieldRelativeSpeed(getChassisSpeed(), getGyroYaw());
        fieldRelativeAccel = new FieldRelativeAccel(fieldRelativeVelocity, lastFieldRelativeVelocity, time-lastTime); //time was 0.02 for standard loop time attempting to calculate it to increase accuracy.  May crash with /0 error if startup is too quick
        lastFieldRelativeVelocity = fieldRelativeVelocity;
        lastTime = time;
    

        SmartDashboard.putNumber("ChassisSpeedX", getChassisSpeed().vxMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeedY", getChassisSpeed().vyMetersPerSecond);
        SmartDashboard.putNumber("ChassisSpeedOmega", getChassisSpeed().omegaRadiansPerSecond);
        SmartDashboard.putNumber("debug/ChassisSpeedX", getChassisSpeed().vxMetersPerSecond);
        SmartDashboard.putNumber("debug/ChassisSpeedY", getChassisSpeed().vyMetersPerSecond);
        SmartDashboard.putNumber("debug/ChassisSpeedOmega", getChassisSpeed().omegaRadiansPerSecond);



        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("debug/Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("debug/Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("debug/Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Robot X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("gyro angle", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("debug/Robot X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("debug/Robot Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("debug/gyro angle", getGyroYaw().getDegrees());



        posePublisher.set(getPose());
        swerveKinematicsPublisher.set(getModuleStates());
        estimatedRobotPosePublisher.set(m_poseEstimator.getEstimatedPosition());
    }
}