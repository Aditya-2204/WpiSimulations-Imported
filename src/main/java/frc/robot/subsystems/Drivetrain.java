// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
  private SwerveModule leftFront;
  private SwerveModule rightFront;
  private SwerveModule leftBack;
  private SwerveModule rightBack;

  private SlewRateLimiter frontLimiter;
  private SlewRateLimiter sideLimiter;
  private SlewRateLimiter turnLimiter;

  private Pigeon2 gyro;

  private RobotConfig config;
  private SwerveDrivePoseEstimator poseEstimator;

  private final NetworkTable drivetrainTable = NetworkTableInstance.getDefault().getTable("Swerve");

  private final NetworkTableEntry FL_velocity = drivetrainTable.getEntry("FL velocity");
  private final NetworkTableEntry FL_angle = drivetrainTable.getEntry("FL angle");

  private final NetworkTableEntry BL_velocity = drivetrainTable.getEntry("FL velocity");
  private final NetworkTableEntry BL_angle = drivetrainTable.getEntry("BL angle");

  private final NetworkTableEntry FR_velocity = drivetrainTable.getEntry("FL velocity");
  private final NetworkTableEntry FR_angle = drivetrainTable.getEntry("FR angle");

  private final NetworkTableEntry BR_velocity = drivetrainTable.getEntry("FL velocity");
  private final NetworkTableEntry BR_angle = drivetrainTable.getEntry("BR angle");

  private static final Drivetrain drivetrain = new Drivetrain();

  public static Drivetrain getInstance(){
    return drivetrain;
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {
  try {
    config = RobotConfig.fromGUISettings();
  }
  catch(Exception e){
    e.printStackTrace();
  }

    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();

    leftFront = new SwerveModule(
      SwerveConstants.LEFT_FRONT_DRIVE_ID, 
      SwerveConstants.LEFT_FRONT_TURN_ID, 
      false, 
      true, 
      SwerveConstants.LEFT_FRONT_CANCODER_ID, 
      SwerveConstants.LEFT_FRONT_OFFSET, 
      false);

    rightFront = new SwerveModule(
      SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
      SwerveConstants.RIGHT_FRONT_TURN_ID, 
      true, 
      true, 
      SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
      SwerveConstants.RIGHT_FRONT_OFFSET, 
      false);

    leftBack = new SwerveModule(
      SwerveConstants.LEFT_BACK_DRIVE_ID, 
      SwerveConstants.LEFT_BACK_TURN_ID, 
      false, 
      true, 
      SwerveConstants.LEFT_BACK_CANCODER_ID, 
      SwerveConstants.LEFT_BACK_OFFSET, 
      false);
    
    rightBack = new SwerveModule(
      SwerveConstants.RIGHT_BACK_DRIVE_ID, 
      SwerveConstants.RIGHT_BACK_TURN_ID, 
      false, 
      true, 
      SwerveConstants.RIGHT_BACK_CANCODER_ID, 
      SwerveConstants.RIGHT_BACK_OFFSET, 
      false);

    frontLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    sideLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

    gyro = new Pigeon2(SwerveConstants.PIGEON_ID);

    poseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.DRIVE_KINEMATICS,
      getHeadingRotation2d(),
      getModulePositions(),
      new Pose2d());

    // AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                 new PIDConstants(1.5, 0.08, .8), // Translation PID constants
    //                 new PIDConstants(1.5, 0.08, .8) // Rotation PID constants
    //         ),
    //         config, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );
  }


@Override
public void periodic() {
    // Get pose from pose estimator
    Pose2d pose = getPose();

    // Log to SmartDashboard
    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
    SmartDashboard.putNumber("Gyro Heading", getHeading());

    // **Update NetworkTables**
    FL_angle.setDouble(leftFront.getAbsoluteEncoderAngle());
    FL_velocity.setDouble(leftFront.getDriveMotorVelocity());

    FR_angle.setDouble(rightFront.getAbsoluteEncoderAngle());
    FR_velocity.setDouble(rightFront.getDriveMotorVelocity());

    BL_angle.setDouble(leftBack.getAbsoluteEncoderAngle());
    BL_velocity.setDouble(leftBack.getDriveMotorVelocity());

    BR_angle.setDouble(rightBack.getAbsoluteEncoderAngle());
    BR_velocity.setDouble(rightBack.getDriveMotorVelocity());
}


  public void swerveDrive(double frontSpeed, double sideSpeed, double turnSpeed, 
    boolean fieldOriented, Translation2d centerOfRotation, boolean deadband){ //Drive with rotational speed control w/ joystick
    if(deadband){
      frontSpeed = Math.abs(frontSpeed) > 0.05 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.05 ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) > 0.05 ? turnSpeed : 0;
    }

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    setModuleStates(moduleStates);
  }

  public void setAllIdleMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void resetAllEncoders(){
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void setHeading(double heading){
    gyro.setYaw(heading);
  }

  public double getHeading(){
    return Math.IEEEremainder(-gyro.getYaw().getValueAsDouble(), 360); //clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = leftFront.getState();
    states[1] = rightFront.getState();
    states[2] = leftBack.getState();
    states[3] = rightBack.getState();
    return states;
  } 

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  }

  public boolean isRedAlliance(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}