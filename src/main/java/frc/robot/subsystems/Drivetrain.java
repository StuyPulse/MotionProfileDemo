/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  private CANSparkMax leftTopMotor; 
  private CANSparkMax leftMidMotor; 
  private CANSparkMax leftBottomMotor;

  private CANSparkMax rightTopMotor; 
  private CANSparkMax rightMidMotor; 
  private CANSparkMax rightBottomMotor;

  private CANEncoder leftEncoder; 
  private CANEncoder rightEncoder; 

  private SpeedControllerGroup leftMotors; 
  private SpeedControllerGroup rightMotors; 

  private AHRS gyro;  

  private DifferentialDriveKinematics kinematics; 
  private DifferentialDriveOdometry odometry; 

  public Drivetrain() {
    leftTopMotor = new CANSparkMax(Constants.Drivetrain.LEFT_TOP_MOTOR_PORT, MotorType.kBrushless); 
    leftMidMotor = new CANSparkMax(Constants.Drivetrain.LEFT_MID_MOTOR_PORT, MotorType.kBrushless); 
    leftBottomMotor = new CANSparkMax(Constants.Drivetrain.LEFT_BOTTOM_MOTOR_PORT, MotorType.kBrushless); 

    rightTopMotor = new CANSparkMax(Constants.Drivetrain.LEFT_TOP_MOTOR_PORT, MotorType.kBrushless); 
    rightMidMotor = new CANSparkMax(Constants.Drivetrain.LEFT_MID_MOTOR_PORT, MotorType.kBrushless); 
    rightBottomMotor = new CANSparkMax(Constants.Drivetrain.LEFT_BOTTOM_MOTOR_PORT, MotorType.kBrushless); 

    leftEncoder = leftTopMotor.getEncoder(); 
    rightEncoder = rightTopMotor.getEncoder(); 

    leftMotors = new SpeedControllerGroup(leftTopMotor, leftMidMotor, leftBottomMotor); 
    rightMotors = new SpeedControllerGroup(rightTopMotor, rightMidMotor, rightBottomMotor); 

    gyro = new AHRS(Port.kMXP); 

    kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.trackWidth); 
    odometry = new DifferentialDriveOdometry(
      new Rotation2d(Math.toRadians(gyro.getAngle()))
    );
  }

  @Override
  public void periodic() {
    updateOdometer();
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);  
    rightMotors.setVoltage(rightVolts); 
  }

  public void stop() {
    tankDriveVolts(0, 0);
  }

  public double getLeftDistance() {
    return leftEncoder.getPosition(); 
  }

  public double getRightDistance() {
    return rightEncoder.getPosition(); 
  }

  public double getLeftVelocity() {
    return leftEncoder.getVelocity(); 
  }

  public double getRightVelocity() {
    return rightEncoder.getVelocity(); 
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0); 
    rightEncoder.setPosition(0); 
  }

  public double getAngle() {
    return gyro.getAngle(); 
  }

  public void resetGyro() {
    gyro.reset();
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics; 
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity()); 
  }

  public void updateOdometer() {
    odometry.update(
      new Rotation2d(Math.toRadians(getAngle())), 
      getLeftDistance(), 
      getRightDistance()
    ); 
  }

  public Pose2d getPose() {
    // the unit is not necessarily meteres 
    // it is whatever you give for distance  
    return odometry.getPoseMeters(); 
  }

  public void zeroOdometer() {
    resetEncoders();
    odometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0)), new Rotation2d(Math.toRadians(getAngle())));
  }

}
