/*
 * This subsytem controls robot vision. It uses the LimelightHelpers class
 * and contains methods that are used to track april tags and gather
 * positional data from them. This data is then used in the swerve
 * subsystem to update the robot's odometry using the pose estimator.
 * 
 * parameters:
 * none
 */


package frc.robot.subsystems;


import frc.lib.util.FieldRelativeAccel;
import frc.lib.util.FieldRelativeSpeed;
import frc.robot.Constants;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.subsystems.Swerve;



public class Eyes extends SubsystemBase {

    // Swerve subsystem for pose estimator
    Swerve s_Swerve;
    Shooter s_Shooter;

    // create objects and variables
    public LimelightHelpers limelight;
    public double tx;
    public double ty;
    public double ta;
    public double tID;
    private double accelerationCompensation = 0.0; //Note this caused a ton of jitter due to inconsistent loop times
    private StructPublisher<Pose2d> posePublisher;
    private StructPublisher<Translation2d> translationPublisher;
    private StructPublisher<Translation2d> trapPublisher;
    public boolean controllerRumble = false;
    public boolean closeToTrap = false;
  
    // constuctor
    public Eyes(Swerve swerve, Shooter shooter) {

        posePublisher = NetworkTableInstance.getDefault().getStructTopic("/Moving Goal pose", Pose2d.struct).publish();
        translationPublisher = NetworkTableInstance.getDefault().getStructTopic("/Moving Goal translation", Translation2d.struct).publish();
        trapPublisher = NetworkTableInstance.getDefault().getStructTopic("/Closest Trap", Translation2d.struct).publish();
        s_Swerve = swerve;
        s_Shooter = shooter;
    }

 
    /*
     * This method will gather all of the positional data of the limelight target.
     * 
     * parameters:
     * none
     * 
     * returns;
     * none
     */
    public void updateData() {

        /* 
         * get data from limelight target
         * tx = x position in degrees in limelight FOV
         * ty = y position in degrees in limelight FOV
         * ta = pitch in degrees in limelight FOV
         * tID = target ID number
         */
        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");
        tID = LimelightHelpers.getFiducialID("");

        // log target data
        SmartDashboard.putNumber("AprilTagX", tx);
        SmartDashboard.putNumber("AprilTagY", ty);
        SmartDashboard.putNumber("AprilTagA", ta);
        SmartDashboard.putNumber("AprilTagID", tID);

    }

    /*
     * This method will wrap all target data into an array for easy access.
     * 
     * Array indexes:
     * [0] = x
     * [1] = y
     * [2] = a (pitch)
     * [3] = ID
     */
    public double[] getDataPackage() {

        double[] data = {
            tx,
            ty,
            ta,
            tID
        };

        return data;
    }

    /*
     * This method will return the pose of the robot based upon the pose of a detected apriltag
     * 
     * parameters:
     * none
     * 
     * returns:
     * robot pose      (Pose2d)
     */
    public Pose2d getRobotPose() {

        Pose2d pose;

        pose = LimelightHelpers.getBotPose2d_wpiBlue("");

        return pose;

    }
    

    /*
     * This method will return the known pose of the desired target.
     * 
     * parameters:
     * none
     * 
     * returns:
     * target pose      (Pose2d)
     */
    public Pose3d getTargetPose() {

        Pose3d pose;

        // if robot is on blue alliance
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {

            // get pose of blue speaker
            pose = new Pose3d(Constants.Positions.speakerBlueX, Constants.Positions.speakerBlueY, 0, new Rotation3d(0,0,Constants.Positions.speakerBlueR));

        // if robot is on red alliance
        } else {

            // get pose of red speaker
            pose = new Pose3d(Constants.Positions.speakerRedX, Constants.Positions.speakerRedY, 0, new Rotation3d(0,0,Constants.Positions.speakerRedR));

        }
        
        return pose;

    }

    /*
     * This method will return the known pose of the desired target.
     * 
     * parameters:
     * none
     * 
     * returns:
     * target pose      (Pose2d)
     */
    public Pose3d getFeedPose() {

        Pose3d pose;

        // if robot is on blue alliance
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {

            // get pose of blue speaker
            pose = new Pose3d(Constants.Positions.ampBlueX, Constants.Positions.ampBlueY, 0, new Rotation3d(0,0,Constants.Positions.ampBlueR));

        // if robot is on red alliance
        } else {

            // get pose of red speaker
            pose = new Pose3d(Constants.Positions.ampRedX, Constants.Positions.ampRedY, 0, new Rotation3d(0,0,Constants.Positions.ampRedR));

        }
        
        return pose;

    }

    public double getTargetRotation() {

        Pose2d robotPose = s_Swerve.m_poseEstimator.getEstimatedPosition();
        Pose3d targetPose = getTargetPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        double angle =  (Math.atan((targetY - robotY) / (targetX - robotX)) * (180 / Math.PI));

        if (robotX > targetX) {

            angle = angle + 180;

        }

        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber(" inverted angle", -angle);

        return -angle + 180;
    }

    public double getFeedRotation() {

        Pose2d robotPose = s_Swerve.m_poseEstimator.getEstimatedPosition();
        Pose3d targetPose = getFeedPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        double angle =  (Math.atan((targetY - robotY) / (targetX - robotX)) * (180 / Math.PI));

        if (robotX > targetX) {

            angle = angle + 180;

        }

        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber(" inverted angle", -angle);

        return -angle + 180;
    }

    public double getMovingTargetRotation() {

        Pose2d robotPose = s_Swerve.m_poseEstimator.getEstimatedPosition();
        Pose2d targetPose = getMovingTarget();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        double angle =  (Math.atan((targetY - robotY) / (targetX - robotX)) * (180 / Math.PI));

        if (robotX > targetX) {

            angle = angle + 180;

        }

        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber(" inverted angle", -angle);

        return -angle + 180;
    }


    public boolean swerveAtPosition(boolean onMove) {
        double error;
        if(onMove){
            error = Math.abs(getMovingTargetRotation() + s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees() % 360) % 360;
        } else { 
            error = Math.abs(getTargetRotation() + s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees() % 360);
        }

        SmartDashboard.putNumber("getMovingTargetRotation", getMovingTargetRotation());
        SmartDashboard.putNumber("estimated rotation", s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees() % 360);
        SmartDashboard.putNumber("rotationError", error);
        //SmartDashboard.putBoolean("swerveAtPosition", error <= Constants.Swerve.atPositionTolerance);

        if (error <= Constants.Swerve.atPositionTolerance) {
            return true;
        } else {
            return false;
        }
    }

    

    public double getShotTime(double distance) {

        double linearSpeed = ((Math.PI * 4 * s_Shooter.m_leftShooter.getVelocity().getValue() * (2 * Math.PI)) / 1/1.375) * 0.0254; //TODO: change shooter gear ratio
        return (distance / linearSpeed) + 0.25; //TODO: tune constant for shot accel/feeding time
    }

    public double getDistanceFromTarget() {

        double distance;

        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {

            double xDistanceToSpeaker = Constants.Positions.speakerBlueX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerBlueY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        } else {

            double xDistanceToSpeaker = Constants.Positions.speakerRedX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerRedY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        }

        return distance;

    }

    public Pose2d getMovingTarget() {
        double shotTime = getShotTime(getDistanceFromTarget());

        Translation2d movingGoalLocation = new Translation2d();

        double robotVelX = s_Swerve.fieldRelativeVelocity.vx;
        double robotVelY = s_Swerve.fieldRelativeVelocity.vy; 

        //TODO calculate accelerations
        double robotAccelX  = s_Swerve.fieldRelativeAccel.ax;
        double robotAccelY = s_Swerve.fieldRelativeAccel.ay;

        double velocityCompensation = 2.0;

        for(int i=0;i<5;i++){

            double virtualGoalX;
            double virtualGoalY;

            if(DriverStation.getAlliance().get() == Alliance.Blue) {
                virtualGoalX = getTargetPose().getX() - shotTime * ((robotVelX * velocityCompensation) + robotAccelX * accelerationCompensation); 
                virtualGoalY = getTargetPose().getY() - shotTime * ((robotVelY * velocityCompensation) + robotAccelY * accelerationCompensation); 
            } else {
                virtualGoalX = getTargetPose().getX() + shotTime * ((robotVelX * velocityCompensation) + robotAccelX * accelerationCompensation); 
                virtualGoalY = getTargetPose().getY() + shotTime * ((robotVelY * velocityCompensation) + robotAccelY * accelerationCompensation); 
            }

            Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);
            translationPublisher.set(testGoalLocation);

            Translation2d toTestGoal = testGoalLocation.minus(s_Swerve.getEstimatedPose().getTranslation());
 
            

            double newShotTime = getShotTime(toTestGoal.getDistance(new Translation2d()));



            if(Math.abs(newShotTime-shotTime) <= 0.010){
                i=4;
            }

            if(i == 4){
                movingGoalLocation = testGoalLocation;
                SmartDashboard.putNumber("NewShotTime", newShotTime);
            }
            else{
                shotTime = newShotTime;
            }

        }
        return new Pose2d(movingGoalLocation, getTargetPose().getRotation().toRotation2d()); //TODO: validate rotation--probably not needed since only get x and y

    }

    public double getDistanceFromMovingTarget() {

        double distance;

        double xDistanceToSpeaker = getMovingTarget().getX() - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
        double yDistanceToSpeaker = getMovingTarget().getY() - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
        distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        return distance;

    }

    public double getDistanceFromTargetBlind() {

        double distance;

        if(DriverStation.getAlliance().get() == Alliance.Blue) {

            double xDistanceToSpeaker = Constants.Positions.speakerBlueX - s_Swerve.getPose().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerBlueY - s_Swerve.getPose().getX();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        } else {

            double xDistanceToSpeaker = Constants.Positions.speakerRedX - s_Swerve.getPose().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerRedY - s_Swerve.getPose().getX();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        }

        return distance;

    }

    public double getDistanceFromTargetAuto() {

        double distance;

        if(DriverStation.getAlliance().get() == Alliance.Blue) {

            double xDistanceToSpeaker = Constants.Positions.speakerBlueX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerBlueY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        } else {

            double xDistanceToSpeaker = Constants.Positions.speakerRedX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerRedY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        }

        return distance;

    }

    public double getDistanceFromTargetTrap(double trapX, double trapY) {

        double xDistance = trapX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
        double yDistance = trapY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
        double distance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        return distance;

    }

    public Pose2d getClosestTrap() {
        
        double trapLeftDistance;
        double trapRightDistance;
        double trapCenterDistance;

        Pose2d closestTrap = new Pose2d();
        double closestDistance;

        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            trapLeftDistance = getDistanceFromTargetTrap(Constants.Positions.blueTrapLeftX, Constants.Positions.blueTrapLeftY);
            trapRightDistance = getDistanceFromTargetTrap(Constants.Positions.blueTrapRightX, Constants.Positions.blueTrapRightY);
            trapCenterDistance = getDistanceFromTargetTrap(Constants.Positions.blueTrapCenterX, Constants.Positions.blueTrapCenterY);

            closestDistance = Math.min(Math.min(trapLeftDistance, trapRightDistance), trapCenterDistance);
            
            if (closestDistance == trapLeftDistance) {
                closestTrap = new Pose2d(Constants.Positions.blueTrapLeftX, Constants.Positions.blueTrapLeftY, Rotation2d.fromDegrees(Constants.Positions.blueTrapLeftR));
            } else if (closestDistance == trapRightDistance){
                closestTrap = new Pose2d(Constants.Positions.blueTrapRightX, Constants.Positions.blueTrapRightY, Rotation2d.fromDegrees(Constants.Positions.blueTrapRightR));
            } else {
                closestTrap = new Pose2d(Constants.Positions.blueTrapCenterX, Constants.Positions.blueTrapCenterY, Rotation2d.fromDegrees(Constants.Positions.blueTrapCenterR));
            }

        } else {
            trapLeftDistance = getDistanceFromTargetTrap(Constants.Positions.redTrapLeftX, Constants.Positions.redTrapLeftY);
            trapRightDistance = getDistanceFromTargetTrap(Constants.Positions.redTrapRightX, Constants.Positions.redTrapRightY);
            trapCenterDistance = getDistanceFromTargetTrap(Constants.Positions.redTrapCenterX, Constants.Positions.redTrapCenterY);

            closestDistance = Math.min(Math.min(trapLeftDistance, trapRightDistance), trapCenterDistance);
            
            if (closestDistance == trapLeftDistance) {
                closestTrap = new Pose2d(Constants.Positions.redTrapLeftX, Constants.Positions.redTrapLeftY, Rotation2d.fromDegrees(Constants.Positions.redTrapLeftR));
            } else if (closestDistance == trapRightDistance){
                closestTrap = new Pose2d(Constants.Positions.redTrapRightX, Constants.Positions.redTrapRightY, Rotation2d.fromDegrees(Constants.Positions.redTrapRightR));
            } else {
                closestTrap = new Pose2d(Constants.Positions.redTrapCenterX, Constants.Positions.redTrapCenterY, Rotation2d.fromDegrees(Constants.Positions.redTrapCenterR));
            }
        }

        if (closestDistance < Constants.Positions.distanceLimit) {
            closeToTrap = true;
        } else {
            closeToTrap = false;
        }

        //Log Target Trap Location
        trapPublisher.set(new Translation2d(closestTrap.getX(), closestTrap.getY()));

        return closestTrap;
    }

    public boolean atTrap() {

        double trapTranslationTolerance = 3.0;
        double trapRotationTolerance = 1.0;

        double xError = Math.abs(getRobotPose().getX() - getClosestTrap().getX());
        double yError = Math.abs(getRobotPose().getY() - getClosestTrap().getY());
        double rError = Math.abs(getRobotPose().getRotation().getDegrees() - getClosestTrap().getRotation().getDegrees());

        if (trapTranslationTolerance >= xError && trapTranslationTolerance >= yError && trapRotationTolerance >= rError) {
            return true;
        } else {
            return false;
        }

    }


    public PathPlannerPath closestTrapPath() {

        Pose2d closestTrap = getClosestTrap();
        Pose2d currentPose = s_Swerve.m_poseEstimator.getEstimatedPosition();
 
        //The bezierFromPoses method required that the rotation component of each pose is the direction of travel, not the rotation of a swerve chassis.  To set the rotation the path should end with, use the GoalEndState.
        
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(

            new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.minus(closestTrap).getRotation()), //TODO Rotation may be wrong
            new Pose2d(closestTrap.getX(), closestTrap.getY(), closestTrap.minus(currentPose).getRotation()) //TODO Rotation may be wrong

        );

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(0.25, 0.25, 2 * Math.PI, 4 * Math.PI), //TODO adjust speeds
            new GoalEndState(0.0, closestTrap.getRotation())
        );

        // Prevent the path from being flipped as all coordinates are blue origin
        path.preventFlipping = true;

        return path;
    }

    // public Command onTheFly() {
    //     PathPlannerPath path = closestTrapPath();
    //     return new FollowPathHolonomic(
    //             path,
    //             () -> s_Swerve.getPose(), // Robot pose supplier
    //             () -> s_Swerve.getChassisSpeed(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //             (speeds) -> s_Swerve.setChassisSpeed(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //             new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //                     new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                     new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //                     Constants.Swerve.maxSpeed, // Max module speed, in m/s
    //                     0.4, // Drive base radius in meters. Distance from robot center to furthest module.
    //                     new ReplanningConfig() // Default path replanning config. See the API for the options here
    //             ),
    //             () -> {
    //               // Boolean supplier that controls when the path will be mirrored for the red alliance
    //               // This will flip the path being followed to the red side of the field.
    //               // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    
    //               var alliance = DriverStation.getAlliance();
    //               if (alliance.isPresent()) {
    //                 return alliance.get() == DriverStation.Alliance.Red;
    //               }
    //               return false;
    //             },
    //             s_Swerve // Reference to this subsystem to set requirements
    //     );
    //   }



    public void updatePoseEstimatorWithVisionBotPose() {
        //invalid limelight
        if (LimelightHelpers.getTV("") == false) {
          return;
        }
        
        // distance from current pose to vision estimated pose
        LimelightHelpers.PoseEstimate lastResult = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
        double fpgaTimestamp = Timer.getFPGATimestamp();
        
        Translation2d translation = s_Swerve.m_poseEstimator.getEstimatedPosition().getTranslation();
        double poseDifference = translation.getDistance(lastResult.pose.getTranslation());
    
          double xyStds;
          double degStds;
          // multiple targets detected
          if (lastResult.tagCount >= 2) {
            xyStds = 0.5;
            degStds = 6;
          }
          // 1 target with large area and close to estimated pose
          else if (lastResult.avgTagArea > 0.8 && poseDifference < 0.5) {
            xyStds = 1.0;
            degStds = 12;
          }
          // 1 target farther away and estimated pose is close
          else if (lastResult.avgTagArea > 0.1 && poseDifference < 0.3) {
            xyStds = 2.0;
            degStds = 30;
          }
          // conditions don't match to add a vision measurement
          else {
            return;
          }
    
          s_Swerve.m_poseEstimator.setVisionMeasurementStdDevs(
              VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
          s_Swerve.m_poseEstimator.addVisionMeasurement(getRobotPose(),
              fpgaTimestamp - (LimelightHelpers.getLatency_Pipeline("")/1000.0) - (LimelightHelpers.getLatency_Capture("")/1000.0));
        }

    public void updatePoseEstimatorWithVisionBotPoseMegatag2() {
        //invalid limelight
        if (LimelightHelpers.getTV("") == false) {
          return;
        }

        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(Math.abs(s_Swerve.gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
          doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
          s_Swerve.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));
          s_Swerve.m_poseEstimator.addVisionMeasurement(
              mt2.pose,
              mt2.timestampSeconds);
        }
    }

    @Override
    public void periodic() {
        s_Swerve.m_poseEstimator.update(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions());


        //updatePoseEstimatorWithVisionBotPose();

        
        if (LimelightHelpers.getTV("") == true) {
            s_Swerve.m_poseEstimator.addVisionMeasurement(
                getRobotPose(), 
                Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("")/1000.0) - (LimelightHelpers.getLatency_Capture("")/1000.0)
            );
        }
        

        SmartDashboard.putNumber("Pose estimator rotations", s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Pose Estimator X", s_Swerve.m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Estimator Y", s_Swerve.m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("target X", getTargetPose().getX());
        SmartDashboard.putNumber("target Y", getTargetPose().getY());
        SmartDashboard.putNumber("Distance to Target", getDistanceFromTarget());

        
        posePublisher.set(getMovingTarget());

    }
}