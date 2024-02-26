// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//package oi.limelightvision.limelight.frc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
//import oi.limelightvision.limelight.frc.ControlMode.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.encoders.CANCoderSwerve;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final CANSparkMax shooter = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax arm_left = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax arm_right = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax feeder = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax intake = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax intake2 = new CANSparkMax(6, MotorType.kBrushless);
  private RobotContainer m_robotContainer;
  private SwerveDrive swerveDrive;
  private final XboxController controller = new XboxController(0);
  private final XboxController controller2 = new XboxController(1);
  private boolean turtleMode = false;
  private final Timer m_timer = new Timer();

  //PID Loop variables
  double lastTime = 0;
  boolean runningPID = false;
  ChassisSpeeds zeroSpeed = new ChassisSpeeds(0, 0, 0);
  double ll_x = 0;
  double ll_y = 0;
  double ll_area = 0;
  double Kp = 0;
  double Ki = 0;
  double command = 0;
  double commandAngle = 0;
  double commandY = 0;
  double commandArm = 0;


  // CANCoderSwerve1 = new CANCoderSwerve(34);
  // CANCoderSwerve2 = new CANCoderSwerve();
  // CANCoderSwerve3 = new CANCoderSwerve();
  // CANCoderSwerve4 = new CANCoderSwerve();



  public double PIDLoop(double sensorReading, double Kp, double Ki, double target, double lastTime) { 
    double elapsedTime = (m_timer.get() - lastTime);
    double error = target - sensorReading;
    double cumulativeError =+ error * elapsedTime;
    command = (Kp * error) + (Ki * cumulativeError);
    lastTime = m_timer.get();
    return command; 
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    }
    catch(IOException i) {

    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*ChassisSpeeds cs_auto = new ChassisSpeeds(1,0,0);
    
     if (timer < 3){
        swerveDrive.driveFieldOriented(cs_auto);
      }
      cs_auto = ChassisSpeeds(0,0,0);
      swerveDrive.driveFieldOriented(cs_auto);
      if (timer > 3.1 && timer < 15) {
        //auto shoot
      }
     
     */
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //CANCoderSwerve(int id); 13 33 36 23
 
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("test", 2);

    //right trigger hold down
    if(controller.getRightBumper()) {
      turtleMode = true;
  } else {
      turtleMode = false;
  }

    //left bumper intake
    if(controller.getLeftBumper() || controller2.getLeftBumper()) {
      System.out.println("Test");
      intake.set(0.5);
      intake2.set(-0.5);
      feeder.set(.5);
    } else {
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
   }

   if (controller2.getLeftTriggerAxis() > .5) {
      feeder.set(1);
   }

    double movementDeadzone = 0.18;
    //0.6,1
    double maxSpeedX = 1.6, maxSpeedY = 1.6, maxSpeedAngle = 1.6;
    double csX = 0, csY = 0, csAngle = 0;
    if(turtleMode) {
      maxSpeedX = 0.2;
      maxSpeedY = 0.2;
      maxSpeedAngle = 0.5;
    }

    if(controller.getLeftY() < -movementDeadzone || controller.getLeftY() > movementDeadzone) {
      runningPID = false;
      csY = controller.getLeftY() * maxSpeedY;
    }

    if(controller.getLeftX() < -movementDeadzone || controller.getLeftX() > movementDeadzone) {
      runningPID = false;
      csX = controller.getLeftX() * maxSpeedX;
    }

    if(controller.getRightX() < -movementDeadzone || controller.getRightX() > movementDeadzone) {
      runningPID = false;
      csAngle = controller.getRightX() * maxSpeedAngle;
    }

    if (controller.getYButton() || controller2.getRightTriggerAxis() > .5) {
      //add feeder power 
      System.out.println("hi");
      shooter.set(1.2);
    } else if (controller2.getLeftY() > .3) {
      shooter.set(.2);
    } else {
      shooter.set(0);
    }


    //Shuffled around the axes to make the actual front the front.
    if(!runningPID) {
      ChassisSpeeds cs = new ChassisSpeeds(csX,-csY,csAngle);
      swerveDrive.driveFieldOriented(cs);
    }
  
    //System.out.println(SwerveMath.calculateMetersPerRotation(0.1, 6.75, 1));
    //System.out.println(SwerveMath.calculateDegreesPerSteeringRotation(21.428, 1));

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //read values periodically
    ll_x = tx.getDouble(0.0);
    ll_y = ty.getDouble(0.0);
    ll_area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", ll_x);
    SmartDashboard.putNumber("LimelightY", ll_y);
    SmartDashboard.putNumber("LimelightArea", ll_area);
    System.out.println("Area: " + ll_area + " Angle: " + ll_x);
 
    if (ll_area <= 0.002) {
      SmartDashboard.putBoolean("Tag?", true);
    } else {
      SmartDashboard.putBoolean("Tag?", false);
    }
  
  
  //Limelight auto subroutines  
 
  // Shooting auto subroutine 
   if (controller.getAButtonPressed()) {
       runningPID = true;
   }

   if (runningPID == true) {
      table.getEntry("pipeline").setValue(0);
      double target_area = .53;
      double target_x = 0;
      double ll_deadzoneA = .01;
      double ll_deadzoneXY = 1.5; 
      //double target_y = 0;
      lastTime = m_timer.get();

    //control forward and back
    if (ll_area < (target_area - ll_deadzoneA) || ll_area > (target_area + ll_deadzoneA)) {
      System.out.println("1");
      Kp = .81;
      Ki = .09;
      PIDLoop(ll_area, Kp, Ki, target_area, lastTime);
      commandY = command;
    } 

  //control left and right   

  if (ll_x < (target_x - ll_deadzoneXY) || ll_x > (target_x + ll_deadzoneXY)) {
       Kp = .020;
       Ki = .002;
       PIDLoop(ll_x, Kp, Ki, target_x, lastTime);
       commandAngle = command;
     }

     //stop if can't see a tag
     if (ll_area <= 0.0002) {
       runningPID = false;
     }

     //drive and stop if command is too low
     if (Math.abs(commandAngle) < .05) {
       commandAngle = 0;
     }

     System.out.println("Command:" + commandY);
     if (Math.abs(commandY) < 0.03) {
      commandY = 0;
     }
     ChassisSpeeds csPID = new ChassisSpeeds(0, -commandY, -commandAngle);
     swerveDrive.driveFieldOriented(csPID);
    
    
    }    

  //Move arm to shoot
  //table.getEntry("pipeline").setValue(1);
   /*   if (ll_y < Math.abs(target_y - ll_deadzoneXY)) {
      //?????????
     } 
     //stop arm
  //shoot and end routine
     if (commandY == 0 && commandAngle == 0 && commandArm == 0 ) {
       //shoot  
       swerveDrive.driveFieldOriented(zeroSpeed);
       runningPID = false;
      }
      */

    //controller 2 commands

    if (controller2.getAButtonPressed() || controller.getXButtonPressed()) {
        //0 arm 
    }

    if (controller2.getYButton()) {
      arm_left.set(0.1);
      arm_right.set(-0.1);
    } else if (controller2.getBButton()) {
      arm_left.set(-0.1);
      arm_right.set(0.1);
    } else {
      arm_left.set(0);
      arm_right.set(0);
    }
 
    if (controller2.getXButton()) {
      // Full arm 
    } 

    

   //reverse intake
   if(controller2.getRightBumper()) {
      intake.set(-.3);
      intake2.set(.3);
      feeder.set(-.3);
    } 



/*  
  //Amp subroutine 
  if (controller.getBButtonPressed()) {
    runningPIDAmp = true;
      //control forward and back
     if (ll_area < (target_area - ll_deadzone) || ll_area > (target_area + ll_deadzone)) {
    
     } 
    //control left and right 
     if (ll_x < (target_x - ll_deadzone) || ll_x > (target_x + ll_deadzone)) {
  
     }
     if (ll_y < (target_y - ll_deadzone) || ll_y > (target_y + ll_deadzone)) {
   
     }
  }
  
  //Make emergency shutoff?
  
 */ 


  }
    

  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
