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
import com.revrobotics.RelativeEncoder;

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
//import edu.wpi.first.util.sendable.Sendable;
//import java.lang.AutoCloseable;
//import com.playingwithfusion.TimeOfFlight;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.arm_left
 */



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final CANSparkMax shooter = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax arm_left = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax arm_right = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax feeder = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax intake = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax intake2 = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax climberLeft = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax climberRight = new CANSparkMax(8, MotorType.kBrushless);
  private final RelativeEncoder leftArmEncoder = arm_left.getEncoder();
  private RobotContainer m_robotContainer;
  private SwerveDrive swerveDrive;
  private final XboxController controller = new XboxController(0);
  private final XboxController controller2 = new XboxController(1);
  private boolean turtleMode = false;
  private final Timer m_timer = new Timer();
  private final Timer s_timer = new Timer();
  private final Timer b_timer = new Timer();
  private final Timer autoTimer = new Timer();
  private boolean isShooting = false;
 // private final TimeOfFlight amogus = new TimeOfFlight(15);

  //PID Loop variables
  double lastTime = 0;
  boolean runningPID = false;
  ChassisSpeeds zeroSpeed = new ChassisSpeeds(0, 0, 0);
  double ll_x = 0;
  double ll_y = 0;
  double ll_area = 0;
  double command = 0;
  double commandAngle = 0;
  double commandY = 0;
  double commandArm = 0;
  double shootTimer = 0;
  double armGoal = 0;
  double Kp2 = 0; 
  double Ki2 = 0;
  double armTarget = 0; 
  boolean armPresetRunning = false;
  double armSetpoint = 0; 
  boolean runningPIDAmp = false;
  boolean hasReached = false;
  boolean teleShoot = false;


  // CANCoderSwerve1 = new CANCoderSwerve(34);
  // CANCoderSwerve2 = new CANCoderSwerve();
  // CANCoderSwerve3 = new CANCoderSwerve();
  // CANCoderSwerve4 = new CANCoderSwerve();



  public double PIDLoop(double sensorReading, double Kp, double Ki, double target) { 
    double elapsedTime = (m_timer.get() - lastTime);
    double error = target - sensorReading;
    double cumulativeError =+ error * elapsedTime;
    command = (Kp * error) + (Ki * cumulativeError);
    lastTime = m_timer.get();
    return command; 
  }

  public void armControl(double armGoal) {
      System.out.println("encoder position" + leftArmEncoder.getPosition());
      Kp2 = .0087;
      Ki2 = .0044;
      armTarget = armGoal; 
      PIDLoop(leftArmEncoder.getPosition(), Kp2, Ki2, armTarget);
      double armCommand = command;
      System.out.println("command " + armCommand);
      if (Math.abs(armGoal - leftArmEncoder.getPosition()) < 1){
        armPresetRunning = false;
      }
      arm_left.set(armCommand);
      arm_right.set(-armCommand);
     
  }

  public void autoShoot() {
      double autoShootTimer = 0;
      if(!hasReached) {
        b_timer.reset();
        autoShootTimer = b_timer.get();
        hasReached = true;
      }
      feeder.set(-.05);
      shooter.set(-.05); 
  //  System.out.println("Timer: " + s_timer.get() + ", Shoot Timer: " + shootTimer);
    
    if(b_timer.get() > (autoShootTimer + .2)) {
      shooter.set(1.3);
    }
    if (b_timer.get() > (autoShootTimer + 1.6)) {
      feeder.set(1.3);
    }
    if(b_timer.get() > (autoShootTimer + 2)) {
      feeder.set(0);
      b_timer.reset();
    }
  }

  public void autoSpeaker() {
    double target_area = .66; //.53
    double target_x = 0;
    double ll_deadzoneA = .01;
    double ll_deadzoneXY = 1.5; 
    //double target_y = 0;
    lastTime = m_timer.get();

    //control forward and back
    if (ll_area < (target_area - ll_deadzoneA) || ll_area > (target_area + ll_deadzoneA)) {
      double Kp = .55; //.81
      double Ki = .00; //.09
      PIDLoop(ll_area, Kp, Ki, target_area);
      commandY = command;
    } 

  //control left and right   

  if (ll_x < (target_x - ll_deadzoneXY) || ll_x > (target_x + ll_deadzoneXY)) {
       double Kp = .000;
       double Ki = .00;
       PIDLoop(ll_x, Kp, Ki, target_x);
       commandAngle = command;
     }

     //stop if can't see a tag
     if (ll_area <= 0.0002) {
       runningPID = false;
     }

     //drive and stop if command is too low
     if (Math.abs(commandAngle) < .1) {
       commandAngle = 0;
       System.out.println("here");
     }

     //System.out.println("Command:" + commandY);
     if (Math.abs(commandY) < 0.07) {
        commandY = 0;
        System.out.println("here2");
     }

     /*if(Math.abs(commandAngle + commandY) < 0.002) {
        teleShoot = true;
     }*/

     ChassisSpeeds csPID = new ChassisSpeeds(0, -commandY, -commandAngle);
     swerveDrive.driveFieldOriented(csPID);
  }

  public void autoAmp() {
    runningPIDAmp = true;
    double target_x = 0; 
    double ll_deadzone = 0;
    double commandX = 0;
    //control left and right 
     if (ll_x < (target_x - ll_deadzone) || ll_x > (target_x + ll_deadzone)) {
      double Kp = .020;
      double Ki = .000;
      PIDLoop(ll_x, Kp, Ki, target_x);
      commandX = command;
     }

     //stop if can't see a tag
     if (ll_area <= 0.0002) {
       runningPIDAmp = false;
     }

     //drive and stop if command is too low
     if (Math.abs(commandX) < .05) {
       commandX = 0;
     }
     ChassisSpeeds csPID = new ChassisSpeeds(commandX, 0, 0);
     swerveDrive.driveFieldOriented(csPID);

     //Auto raise arm
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

    autoTimer.start();
    b_timer.start();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    ChassisSpeeds cs_auto = new ChassisSpeeds(0,.3,0);
    /*Auto Options:
     * 1:Shoot, Drive back
     * 2:Shoot, drive back, pick up another ring
     * 3:Shoot, drive back, pick up ring, shoot again
     * 4:Drive back
     */


    //Auto option 1: Shoot, drive back
    /*
    if (autoTimer.get() < 3){
        autoShoot();
    } else if (autoTimer.get() < 5){
       swerveDrive.driveFieldOriented(cs_auto);
    } else {
      swerveDrive.driveFieldOriented(zeroSpeed);
    }
     
     */

   /* //Auto option 2: Shoot, drive back, pick up another ring
      if (autoTimer.get() < 3){
        autoShoot();
      } else if (autoTimer.get() < 6){
        swerveDrive.driveFieldOriented(cs_auto);
        intake.set(0.5);
        intake2.set(-0.5);
        feeder.set(.5);
      } else {
        swerveDrive.driveFieldOriented(zeroSpeed);
        intake.set(0);
        intake2.set(0);
        feeder.set(0);
      }

    */

     //Auto option 3: Shoot, drive back, pick up ring, shoot again
    /*
     if (autoTimer.get() < 3){
        autoShoot();
      } else if (autoTimer.get() < 6){
        swerveDrive.driveFieldOriented(cs_auto);
        intake.set(0.5);
        intake2.set(-0.5);
        feeder.set(.5);
      } else if (autoTimer.get() < 6.5) {
        swerveDrive.driveFieldOriented(zeroSpeed);
        intake.set(0);
        intake2.set(0);
        feeder.set(0);
      } 

      ////!!!!!!! either shoot with subroutine or drive foward and autoshoot
     */

    //Auto option 3: Drive
    /*
    if (autoTimer.get() < 3){
       swerveDrive.driveFieldOriented(cs_auto);
    } else {
      swerveDrive.driveFieldOriented(zeroSpeed);
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
    s_timer.start();
    b_timer.start();
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    System.out.println("encoder " + leftArmEncoder.getPosition());
    //To get Limelight Data
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //read values periodically
    ll_x = tx.getDouble(0.0);
    ll_y = ty.getDouble(0.0);
    ll_area = ta.getDouble(0.0);
    table.getEntry("pipeline").setValue(0);
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

    //right trigger hold down for turtle mode
    if(controller.getRightBumper()) {
      turtleMode = true;
  } else {
      turtleMode = false;
  }
  
    //left bumper intake //Press hold // change to auto shutoff once sensor is attatched
    if(controller.getLeftBumper() || controller2.getLeftBumper()) {
      intake.set(0.5);
      intake2.set(-0.5);
      feeder.set(.5);
    } 
    else if(isShooting) {
      intake.set(0);
      intake2.set(0);
    }
    else {
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
    }

    double movementDeadzone = 0.3;
    double maxSpeedX = 1.6, maxSpeedY = 1.6, maxSpeedAngle = 1.6;
    double csX = 0, csY = 0, csAngle = 0;
    if(turtleMode) {
      maxSpeedX = 0.2;
      maxSpeedY = 0.2;
      maxSpeedAngle = 0.5;
    }

    if(controller.getLeftY() < -movementDeadzone || controller.getLeftY() > movementDeadzone) {
      //runningPID = false;
      //runningPIDAmp = false;
      csY = controller.getLeftY() * maxSpeedY;
    }

    if(controller.getLeftX() < -movementDeadzone || controller.getLeftX() > movementDeadzone) {
      //runningPID = false;
      //runningPIDAmp = false;
      csX = controller.getLeftX() * maxSpeedX;
    }

    if(controller.getRightX() < -movementDeadzone || controller.getRightX() > movementDeadzone) {
      //runningPID = false;
      //runningPIDAmp = false;
      csAngle = controller.getRightX() * maxSpeedAngle;
    }

    if ((controller2.getRightTriggerAxis() > 0.3) || controller.getAButton()) {
      s_timer.reset();
      feeder.set(-.05);
      shooter.set(-.05);
      shootTimer = s_timer.get();
      isShooting = true;
     // teleShoot = false;
    } else if (controller.getXButton()) {
      shooter.set(.4);
    } 
    else if(isShooting) {
      
    }
    else {
      shooter.set(0);
    }
  //  System.out.println("Timer: " + s_timer.get() + ", Shoot Timer: " + shootTimer);
    
    if(s_timer.get() > (shootTimer + .2) && isShooting) {
      shooter.set(1.3);
    }
    if (s_timer.get() > (shootTimer + 1.6) && isShooting) {
      feeder.set(1.3);
    }
    if(s_timer.get() > (shootTimer + 2) && isShooting) {
      feeder.set(0);
      isShooting = false;
      s_timer.reset();
    }
    

   
    //Shuffled around the axes to make the actual front the front.
    if(!runningPID || !runningPIDAmp) {
      ChassisSpeeds cs = new ChassisSpeeds(csX,-csY,csAngle);
      swerveDrive.driveFieldOriented(cs);
    }
  
    //System.out.println(SwerveMath.calculateMetersPerRotation(0.1, 6.75, 1));
    //System.out.println(SwerveMath.calculateDegreesPerSteeringRotation(21.428, 1));

  if (controller.getLeftTriggerAxis() > .3) {
    climberLeft.set(.1);
    climberRight.set(.1);
  } else if (controller.getRightTriggerAxis() > .3){
    climberLeft.set(-.1);
    climberRight.set(-.1);
  } else {
    climberLeft.set(0);
    climberRight.set(0);
  }

    if (controller2.getAButtonPressed() || controller.getYButtonPressed()) {
        //0 arm
        armSetpoint = 0;
        armPresetRunning = true;
    }

     if (controller.getBButtonPressed()) {
      armSetpoint = 31;
      armPresetRunning = true;
    // runningPIDAmp = true;  
   }

    if (armPresetRunning) {
      armControl(armSetpoint);
    }

    if (controller2.getYButton()) {
      armPresetRunning = false;
      arm_left.set(0.1);
      arm_right.set(-0.1);
    } else if (controller2.getBButton()) {
      armPresetRunning = false;
      arm_left.set(-0.1);
      arm_right.set(0.1);
    } else if (!armPresetRunning) {
      arm_left.set(0);
      arm_right.set(0);
    }
    else {

    }

   //reverse intake
   if(controller2.getRightBumper()) {
      feeder.set(-.1);
      shooter.set(-.1);
    } 


    if (controller2.getLeftTriggerAxis() > .3){
      intake.set(-.1);
      intake2.set(.1);
    }

  
  //Limelight auto subroutines  
 
  // Shooting auto subroutine 
   /*if (controller.getAButtonPressed()) {
       runningPID = true;
   }*/
  /*  if (runningPID == true) {
      autoSpeaker();
   }   */
   
   
   /*if(runningPIDAmp) {
      autoAmp();
   }*/

  //Move arm to shoot
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

    // change back to pressed
    // 31.4 top, 0.25 bottom

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
