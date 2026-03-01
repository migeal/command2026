// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.Motorconstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants.Motorconstants;
public class Loader extends SubsystemBase {
   WPI_VictorSPX load = new WPI_VictorSPX(Motorconstants.loadMotor);
  /** Creates a new ExampleSubsystem. */
  public Loader() {
    load.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void reload(){
    load.set(0.5);
  }
  //just in case
  public void deStick(){
    load.set(-1);
  }
  public void stop(){
    load.set(0);
  }
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
