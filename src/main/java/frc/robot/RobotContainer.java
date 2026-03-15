// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import org.littletonrobotics.conduit.schema.Joystick;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;

import frc.robot.commands.ReLoad;
import frc.robot.commands.DeStick;
import frc.robot.commands.Eat;
import frc.robot.commands.Spit;
import frc.robot.commands.Fire;

import frc.robot.Constants.DriveConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      private final CommandJoystick m_GunnerStick = new CommandJoystick(OperatorConstants.kGunnerControllerPort);
private final GenericHID m_pray = new GenericHID(0);
      //
      //subsystems 

      private final DriveTrain m_robotDrive = new DriveTrain();
      private final Loader m_loader = new Loader();
      private final Pickup m_pickup = new Pickup();
      private final Shooter m_shooter = new Shooter();

      //Commands
      private final ReLoad m_reLoad = new ReLoad(m_loader);
       private final DeStick m_deStick = new DeStick(m_loader);
       private final Eat m_eat = new Eat(m_pickup);
       private final Spit m_spit = new Spit(m_pickup);
       private final Fire m_fire = new Fire(m_shooter);
       //buttons

       private final JoystickButton m_trigger = new JoystickButton(m_pray,1);
       private final JoystickButton m_reload = new JoystickButton(m_pray,2);
       private final JoystickButton m_unload = new JoystickButton(m_pray,5);
       private final JoystickButton m_reload_w_pickup = new JoystickButton(m_pray, 3);
         
       private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

   
     //Configure driving default
      m_robotDrive.setDefaultCommand(
        // Forward motion controls x speed (forward), sideways motion controls y speed (sideways).
          new RunCommand (  
            () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(-m_driverController.getLeftY(), DriveConstants.kDriveDeadband),
              -MathUtil.applyDeadband(-m_driverController.getLeftX(), DriveConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getRightX(), DriveConstants.kDriveDeadbandZ),
              DriveConstants.kTeleField), m_robotDrive)
                 
          );

           autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
     SmartDashboard.putData("test fire", new PathPlannerAuto("test fire"));

     SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    

    m_driverController.leftTrigger(0.7).whileTrue(m_spit);

    m_driverController.rightTrigger(0.7).whileTrue(m_eat);

    m_GunnerStick.button(1).toggleOnTrue(m_fire);
    m_trigger.toggleOnTrue(m_fire);
if(m_reload.getAsBoolean()){
    m_driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
   }
   else{
    m_driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
   }
   
    m_GunnerStick.button(2).whileTrue(m_reLoad);

    m_GunnerStick.button(5).whileTrue(m_deStick);

    m_GunnerStick.button(3).whileTrue(m_reLoad);

    m_GunnerStick.button(3).whileTrue(m_eat);

    m_reload.whileTrue(m_reLoad);

    m_unload.whileTrue(m_deStick);

    m_reload_w_pickup.whileTrue(m_reLoad);
    m_reload_w_pickup.whileTrue(m_eat);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return autoChooser.getSelected();
  }
}
