/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainFollowCommand extends CommandBase {
  Drivetrain drivetrain; 

  // What other things do we need for this command? (See hints)
  public DrivetrainFollowCommand(Drivetrain drivetrain) {
    this.drivetrain = drivetrain; 
    // HINT: timer? 
    // HINT: trajectories? 
    // don't forget there are two sides. how do we find them?
    // HINT: controllers? constants? 
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // reset and start the timer
    // reset the controllers
    // reset sensors
  }

  @Override
  public void execute() {
    // use tank drive volts  
  }

  public double getLeftVoltage() {
    // use FF and PID for left side 
  }

  public double getRightVoltage() {
    // use FF and PID for right side 
  }

  @Override
  public void end(boolean interrupted) {
    // stop the timer
    // stop the drivetrain 
  }

  @Override
  public boolean isFinished() {
    // how can we decide when the path is finished?
    // HINT: use time 
    return false;
  }
}
