/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private Drivetrain drivetrain; 
  
  private SendableChooser<Command> chooser; 

  public RobotContainer() {
    drivetrain = new Drivetrain(); 

    // create your trajecotry here
    // HINT: what class stores physics stuff? 
    // HINT: trajectory does not have it's own constructor. What class is used to make it? 

    // Configure the button bindings
    configureButtonBindings();
    // Add autons to chooser 
    addAutons();
  }

  private void configureButtonBindings() {
  }

  private void addAutons() {
    chooser = new SendableChooser<>(); 
    chooser.setDefaultOption("Example Command", new ExampleCommand(m_exampleSubsystem));
    // add motion profile command here
    // HINT: use the command and trajectory we created 
    // chooser.addOption("Ramsete Command", getRamseteCommand(trajectory));
    SmartDashboard.putData(chooser);
  }

  /*
  private RamseteCommand getRamseteCommand(Trajectory trajectory) {
    SimpleMotorFeedforward FFController = new SimpleMotorFeedforward(
      Constants.Drivetrain.FF.ks, Constants.Drivetrain.FF.kv, Constants.Drivetrain.FF.ka); 
    PIDController PIDController = new PIDController(
      Constants.Drivetrain.PID.kp, Constants.Drivetrain.PID.ki, Constants.Drivetrain.PID.kd); 
    RamseteCommand ramsete = new RamseteCommand(
      trajectory, 
      drivetrain::getPose, 
      new RamseteController(), 
      FFController, 
      drivetrain.getKinematics(), 
      drivetrain::getWheelSpeeds, 
      PIDController, 
      PIDController, 
      drivetrain::tankDriveVolts, 
      drivetrain); 
    return ramsete; 
  }
  */

  public Command getAutonomousCommand() {
    return chooser.getSelected(); 
  }
}
