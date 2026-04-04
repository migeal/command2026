// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.ControlSystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;


public class DriveTrain extends SubsystemBase {
  private boolean MTOne = false;
  
  private final Field2d m_field = new Field2d();
  /** Creates a new Drive Train Subsystem. */

  


  private final Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.WheelXdist, DriveConstants.WheelYdist);
  private final Translation2d m_frontRightLocation = new Translation2d(DriveConstants.WheelXdist, -DriveConstants.WheelYdist);
  private final Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.WheelXdist, DriveConstants.WheelYdist);
  private final Translation2d m_backRightLocation = new Translation2d(-DriveConstants.WheelXdist, -DriveConstants.WheelYdist);

  private final SwerveModule m_frontLeft= new SwerveModule(
    ControlSystem.kLeftFrontDrive,
    ControlSystem.kLeftFrontTurn, 
    ControlSystem.kLFturn, 
    DriveConstants.kFrontLeftModuleAngularOffset);
    //DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
    ControlSystem.kRightFrontDrive,
    ControlSystem.kRightFrontTurn, 
    ControlSystem.kRFturn,
    DriveConstants.kFrontRightModuleAngularOffset);
    //DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_backLeft = new SwerveModule(
    ControlSystem.kLeftBackDrive,
    ControlSystem.kLeftBackTurn, 
    ControlSystem.kLBturn,
    DriveConstants.kBackLeftModuleAngularOffset);
    //DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_backRight = new SwerveModule(
    ControlSystem.kRightBackDrive,
    ControlSystem.kRightBackTurn, 
    ControlSystem.kRBturn,
    DriveConstants.kBackRightModuleAngularOffset);
    //DriveConstants.kBackRightChassisAngularOffset);


public void configureAutoBuilder() {
    try{
      RobotConfig config = RobotConfig.fromGUISettings();

      

      // Configure AutoBuilder
      AutoBuilder.configure(
        m_odometry::getEstimatedPosition, 
        m_odometry::resetPose, 
        RobotContainer::getSpeeds,
        this::DriveRobotRelative,
        new PPHolonomicDriveController(
          Constants.DriveConstants.translationConstants,
          Constants.DriveConstants.rotationConstants
        ),
        config,
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
        this
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
    
  }

  //private final ADIS16448_IMU m_imu = new ADIS16448_IMU();
  // Might be a point of error, hope for the best though.
  private final AHRS m_imu = new AHRS(NavXComType.kMXP_SPI);
  
    /***********************************************************************
     * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
     * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
     * 
     * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
     * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
     * 
     * VMX-pi: - Communication via USB. - See
     * https://vmx-pi.kauailabs.com/installation/roborio-installation/
     * 
     * Multiple navX-model devices on a single robot are supported.
     ************************************************************************/
    
  
  private final SwerveDriveKinematics m_kinematics =
    new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    
        
  public final SwerveDrivePoseEstimator m_odometry =
    new SwerveDrivePoseEstimator(
      m_kinematics,
      new Rotation2d(-m_imu.getAngle()*Math.PI/180),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      },
      new Pose2d()
      );

  public DriveTrain() {}

  public void DriveRobotRelative(ChassisSpeeds robotReletiveSpeeds){
   m_frontLeft.driveRobotRelative(robotReletiveSpeeds);
   m_frontRight.driveRobotRelative(robotReletiveSpeeds);
   m_backLeft.driveRobotRelative(robotReletiveSpeeds);
   m_backRight.driveRobotRelative(robotReletiveSpeeds);

    
  }
  

  @Override
  public void periodic() {
    
    SignalLogger.enableAutoLogging(false);
   // LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight");
    // update odometry
    m_odometry.update(
        Rotation2d.fromDegrees(-m_imu.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
   
    //boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    //if(useMegaTag2 == false)
        //System.out.println("Limelight code run");
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-ronin");
       Double[] mt1Pos =  {(mt1.pose.getX()),(mt1.pose.getY()),(mt1.pose.getRotation().getDegrees())};
      if(mt1 != null){
        //System.out.println("mt1 not null");
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
            MTOne = true;
            //System.out.println("tagcount==1");

            //System.out.println("mt1 is 1");
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        //System.out.println("mt1 == 0");
        doRejectUpdate = true;
        MTOne = false;
      }

      if(!doRejectUpdate)
      {
        //System.out.println("Update successful");
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_odometry.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
            //System.out.println("m_odometry added");
      }
    }
    
    m_field.setRobotPose(m_odometry.getEstimatedPosition());
    //System.out.println("m_field updated to odometry");
   
    // if mt1 is more than 0 
    SmartDashboard.putBoolean("MT1 = one", MTOne);
     // smartdash pos
    SmartDashboard.putData("odomitry pos", m_field);
    
    SmartDashboard.putNumberArray("mti Pose", mt1Pos);
    // Put values to SmartDashboard 
    SmartDashboard.putNumber("Front Left Drive Speed", DriveVelFL());
    SmartDashboard.putNumber("Front Right Drive Speed", DriveVelFR());
    SmartDashboard.putNumber("Back Left Drive Speed", DriveVelBL());
    SmartDashboard.putNumber("Back Right Drive Speed", DriveVelBR());

    //Display Odometry IMU angle
    SmartDashboard.putNumber("IMU Angle", getIMUAngle());
    SmartDashboard.putBoolean("isConnected", m_imu.isConnected());
    SmartDashboard.putBoolean("isCalibrating", m_imu.isCalibrating());
    SmartDashboard.putBoolean("isMoving", m_imu.isMoving());
    SmartDashboard.putBoolean("isRotating", m_imu.isRotating());
    
    //Display Kinematics
    SmartDashboard.putNumber("Front Left Encoder Count", TurnCountFL());
    SmartDashboard.putNumber("Back Left Encoder Count", TurnCountBL());
    SmartDashboard.putNumber("Front Right Encoder Count", TurnCountFR());
    SmartDashboard.putNumber("Back Right Encoder Count", TurnCountBR());

     //Display Wheel orientations
     SmartDashboard.putNumber("FL Wheel Angle", wheelAngleFL());
     SmartDashboard.putNumber("FR Wheel Angle", wheelAngleFR());
     SmartDashboard.putNumber("BL Wheel Angle", wheelAngleBL());
     SmartDashboard.putNumber("BR Wheel Angle", wheelAngleBR());
      
     //Display Wheel orientations
     SmartDashboard.putNumber("FL NEO Wheel Angle", wheelAngleNEOFL());
     SmartDashboard.putNumber("FR NEO Wheel Angle", wheelAngleNEOFR());
     SmartDashboard.putNumber("BL NEO Wheel Angle", wheelAngleNEOBL());
     SmartDashboard.putNumber("BR NEO Wheel Angle", wheelAngleNEOBR());
  }

  public final double getIMUAngle() {
    //System.out.printf("Odo Angle Call %f\n", m_imu.getAngle());
    double iMUAngle = -m_imu.getAngle()*Math.PI/180;
    return iMUAngle;
  } 
  
  /**
   * Method to drive the robot using the Joystick.
   * 
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param zRot Angular rotation of the robot by twisting the Joystick.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double zRot, boolean fieldRelative) {
    
    double xSpeedCommanded = xSpeed;
    double ySpeedCommanded = ySpeed;
    double m_currentRotation = zRot;


    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeed;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeed;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    
    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putNumber("Z Rot ", zRot);
    SmartDashboard.putBoolean("Field Relative ", fieldRelative);
    //if(xSpeed + ySpeed != 0) {System.out.printf("Field %b, x=%f, y=%f, rot=%f\n", fieldRelative, xSpeed, ySpeed, zRot);}

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_imu.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeed);
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]); 

    //System.out.printf("Module state 0 output %f", m_frontLeft.getPosition().angle.getDegrees());
    //System.out.printf("Module state 0 calc%f \n", swerveModuleStates[0].angle.getDegrees());
  }
    
  public void resetEncoders() {
    //m_frontLeft.resetEncoders();
    //m_backLeft.resetEncoders();
    //m_frontRight.resetEncoders();
    //m_backRight.resetEncoders();
  }

  // Measure turing encoder counts
  public double TurnCountFR() {
    double turningOut = m_frontRight.TurnOutput();
    return turningOut;
  }

  public double TurnCountFL() {
    double turningOut = m_frontLeft.TurnOutput();
    return turningOut;
  }

  public double TurnCountBR() {
    double turningOut = m_backRight.TurnOutput();
    return turningOut;
  }

  public double TurnCountBL() {
    double turningOut = m_backLeft.TurnOutput();
    return turningOut;
  }

  // Measure driving wheel speeds
  public double DriveVelFL() {
    double driveVelFL = m_frontLeft.DriveOutput();
    return driveVelFL;
  }

  public double DriveVelFR() {
    double driveVelFR = m_frontRight.DriveOutput();
    return driveVelFR;
  }

  public double DriveVelBL() {
    double driveVelBL = m_backLeft.DriveOutput();
    return driveVelBL;
  }

  public double DriveVelBR() {
    double driveVelBR = m_backRight.DriveOutput();
    return driveVelBR;
  }

  // Calculate wheel angles
  public double wheelAngleFL() {
    double angle = m_frontLeft.wheelAngle();
    return angle;
  }
  public double wheelAngleFR() {
    double angle = m_frontRight.wheelAngle();
    return angle;
  }
  public double wheelAngleBL() {
    double angle = m_backLeft.wheelAngle();
    return angle;
  }
  public double wheelAngleBR() {
    double angle = m_backRight.wheelAngle();
    return angle;
  }

  // Calculate wheel angles
  public double wheelAngleNEOFL() {
    double angle = m_frontLeft.getTurnAngle();
    return angle;
  }
  public double wheelAngleNEOFR() {
    double angle = m_frontRight.getTurnAngle();
    return angle;
  }
  public double wheelAngleNEOBL() {
    double angle = m_backLeft.getTurnAngle();
    return angle;
  }
  public double wheelAngleNEOBR() {
    double angle = m_backRight.getTurnAngle();
    return angle;
  }

  public double Distance() {
    var Distance = m_backLeft.distance();
    return Distance;
  }

/*
  public void DriveForward() {
    m_backLeft.DriveForward();
    m_frontLeft.DriveForward();
    m_backRight.DriveForward();
    m_frontRight.DriveForward();
  }
*/
  public void DriveStop() {
    m_backLeft.DriveStop();
    m_frontLeft.DriveStop();
    m_backRight.DriveStop();
    m_frontRight.DriveStop();
  }

  // @Override
  // public void periodic() {

  //   // This method will be called once per scheduler run during simulation

  //  // topRightAngle.append(m_frontRight.getAngle());
  // }
  
}