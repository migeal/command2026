// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.Motorconstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants.Motorconstants; 
public class Shooter extends SubsystemBase {
  WPI_VictorSPX LaunchR = new WPI_VictorSPX(Motorconstants.ShootMotorR);
  WPI_VictorSPX LaunchL = new WPI_VictorSPX(Motorconstants.ShootMotorL);
  /** Creates a new ExampleSubsystem. */
  public Shooter() {}

  
 
//shoot ball
  public void fire(){
   LaunchR.set(0.535);
    LaunchL.set(0.535);
  }
  //just in case
  public void HalfFire(){
   LaunchR.set(0.5);
    LaunchL.set(0.5);
  }
  public void reversefire(){
   LaunchR.set(-1);
    LaunchL.set(-1);
  }
  //stop shooting
  public void stop(){
   LaunchR.set(0);
    LaunchL.set(0);
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
