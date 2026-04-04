// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveAuto extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_subsystem;
 private final Timer m_time;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveAuto(DriveTrain subsystem) {
    m_subsystem = subsystem;
    m_time = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time.reset();
    m_time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_time.get() < 5) {
      m_subsystem.drive(-2, 0, 0, isScheduled());
    } else if (m_time.get() > 5) {
      m_subsystem.drive(0, 0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
