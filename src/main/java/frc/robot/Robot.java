// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// package oi.limelightvision.limelight.frc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Notifier;
// import oi.limelightvision.limelight.frc.ControlMode.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.sensors.CANCoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.encoders.CANCoderSwerve;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
// for LL3 localization
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
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
<<<<<<< Updated upstream
=======
  private AHRS gyro = null;
>>>>>>> Stashed changes

  private final XboxController controller = new XboxController(0);
  private final XboxController controller2 = new XboxController(1);
  private boolean turtleMode = false;
  private final Timer m_timer = new Timer();
  private final Timer s_timer = new Timer();
  private final Timer b_timer = new Timer(); // is this used?
  private final Timer t_timer = new Timer(); // teleshoot timer
  private final Timer autoTimer = new Timer();
  private boolean isShooting = false;
  private double autoShootTimer = 0;
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
  double shooterProxy = 0;
  boolean armPresetRunning = false;
  double armSetpoint = 0; 
  boolean runningPIDAmp = false;
  boolean hasReached = false;
  boolean teleShootBool = false;
  double maximumSpeed;
  boolean hasReached2 = false;
  boolean hasReached3 = false;
  boolean hasReached4 = false;
  double ll_f_x = 0;
  double ll_f_y = 0;
  double ll_f_area = 0;
  //NetworkTableEntry tx, ty, ta, tx_f, ty_f, ta_f;


  // CANCoderSwerve1 = new CANCoderSwerve(34);
  // CANCoderSwerve2 = new CANCoderSwerve();
  // CANCoderSwerve3 = new CANCoderSwerve();
  // CANCoderSwerve4 = new CANCoderSwerve();


  public double motorRotationsToDegreesArm(double rotationReading) {
    double degreeValue = (16*360*rotationReading)/(72*20);
    return degreeValue;
  }

  public double PIDLoop(double sensorReading, double Kp, double Ki, double target) { 
    double elapsedTime = (m_timer.get() - lastTime);
    double error = target - sensorReading;
    double cumulativeError =+ error * elapsedTime;
    command = (Kp * error) + (Ki * cumulativeError);
    lastTime = m_timer.get();
    return command; 
  }
//most of this needs to get tuned 
  public void armControl(double armGoal) {
    //Arm Up Speed //should be faster 

    
    armGoal = armGoal + 0.375 + 0.15; // offset is to account for arm error
    if (armGoal > leftArmEncoder.getPosition()) { // add some max value / clamp
      Kp2 = .07; 
      Ki2 = 0;//.07;
      armTarget = armGoal; 
      PIDLoop(leftArmEncoder.getPosition(), Kp2, Ki2, armTarget);
      double armCommand = command;
      
      // System.out.println(command);

      if (Math.abs(armGoal - leftArmEncoder.getPosition()) < 1){
        armPresetRunning = false;     
      }
      if (leftArmEncoder.getPosition() > 31.5) {
        armPresetRunning = false;   
      }

      arm_left.set(armCommand);
      arm_right.set(-armCommand);
      System.out.println("Command: " + armCommand);
      System.out.println("Encoder: " + leftArmEncoder.getPosition());

      //System.out.println("command: " + armCommand);
      //CLAMP THE COMMAND
    }
    //Arm Down Speed //should be slower
    if (armGoal < leftArmEncoder.getPosition()) {
      Kp2 = .009;
      Ki2 = .0044;
      armTarget = armGoal; 
      PIDLoop(leftArmEncoder.getPosition(), Kp2, Ki2, armTarget);
      double armCommand = command;
      if (Math.abs(armGoal - leftArmEncoder.getPosition()) < 1){
        armPresetRunning = false;     
      }
      if (leftArmEncoder.getPosition() > .2) {
        armPresetRunning = false;   
      }
      // System.out.println("arm command: " + armCommand);
      arm_left.set(armCommand);
      arm_right.set(-armCommand);
      System.out.println("Command: " + armCommand);
      System.out.println("Encoder: " + leftArmEncoder.getPosition());
    }
  }

  public void teleShoot() { // new shooting macro
    // System.out.println("1 t_timer: " + t_timer.get());
    // System.out.println("HR: " + hasReached3);
    
    if(hasReached3 == false) {
      t_timer.reset();
      hasReached3 = true;
    }
    
    // System.out.println("2 t_timer: " + t_timer.get());
    if(0 < t_timer.get() && t_timer.get() < 0.3) { // move note back
      feeder.set(-.1);
      shooter.set(0.1);
    }
    if(0.3 < t_timer.get() && t_timer.get() < 0.9) { // spool shooter
                                          // I updated this from 0.5s to 0.6s for reliability
      shooter.set(-0.8);
    }
    if(0.9 < t_timer.get() && t_timer.get() < 1.25) { // fire
      feeder.set(1);
    }
    if(1.25 < t_timer.get() && t_timer.get() < 1.5) {
      armSetpoint = 0;
      armPresetRunning = true;
    }
    
    if(t_timer.get() < 2.8) {

    }
    else {
      t_timer.reset();
      hasReached3 = false;
    }
  }

  public void autoShoot() {
     // replace this with the new teleshoot function
  }

  // limelight 3 aiming by jake pt 1 :)________________________________________________________

  //mapping thing from stackoverflow :)
  final static double EPSILON = 1e-12;

  public static double map(double valueCoord1,
          double startCoord1, double endCoord1,
          double startCoord2, double endCoord2) {
  
      if (Math.abs(endCoord1 - startCoord1) < EPSILON) {
          throw new ArithmeticException("/ 0");
      }
  
      double offset = startCoord2;
      double ratio = (endCoord2 - startCoord2) / (endCoord1 - startCoord1);
      return ratio * (valueCoord1 - startCoord1) + offset;
  }

  public static double sigmoidMap(double value, double tuningParameter) {
    double mappedValue = 0;
    mappedValue = 1 / (1 + Math.exp(-value*tuningParameter));
    return mappedValue;
  }

  public double limelight_arm_aim_proportional() {

    // read distance with ta or ty --> ty should be more reliable because we will also rotate
    // set the arm to some value based on that
    double targetAngle = 0;
    // total arm range is 0-31
    // horizontal value is 16
    // area is 0.67 -> .18 (don't use this)
    // ty is 9.2 -> -23 (use this)
    // ty now 0 -> -30 bc of ll3 crosshair tuning

    // map ta/ty to arm
    // tune map values
    // also I don't think that the physics are linear, soooooo... ( we need a sin curve on this)
    // targetAngle = map(ll_area, 0.67, .18, 0, 8);
    
    /*
    targetAngle = map(ll_y, 9.2, -23, 0, 8); // end coord 2 is what you tune. affects how steep arm is at furthest point
    // is 7 a good value?
    */

    // target angle using sigmoid (not linear)
    // double bounds = 1; // tune bounds
    // double sigmoidTuner = 3; // tune sigmoid. should be at least 5???
    // double endVal = 7.65;

    // targetAngle = map(ll_y, 0, -30, -bounds, bounds); 
    // targetAngle = sigmoidMap(targetAngle, sigmoidTuner);
    // targetAngle = map(targetAngle, 0, 1, 0, endVal); // tune this end coord if shot not going on from farthest point

    // targetAngle = map(ll_y, 0, -30, 0, endVal); // tune this end coord if shot not going on from farthest point

    // target angle using trig (because physics)
    // targetAngle = map(ll_y, 0, -30, 0, 3.14/2);

    //proper process:
      // measure delta between tag and crosshair (ty)
      // use ty and limelight mount angle to find horizontal part of triangle
      // use that horizontal triangle value (the base) in an arcsin function to get an angle
      // map that angle to our arm
      // tune the map endpoints and physics should do the rest
    
    // ll_y
    // theta = 37.876 deg

    double theta = Math.toRadians(90-27.876); // limelight mount angle (measured)
    // double psi = Math.asin( ((ll_y) / (Math.tan(Math.toRadians(theta)))) );
    // targetAngle = map(Math.toDegrees(psi), -(3.14)/2, (3.14)/2, 0, 7.65); 
    // tune this end coord if shot not going in from farthest point

    // this little section is in inches
    double aprilTagHeight = 57.235; //61.375;
    // double y = aprilTagHeight + (Math.abs(ll_y))*2;

    double y = aprilTagHeight;
    theta = theta + Math.toRadians(ll_y);

    double x = y / (Math.tan(theta)) + 10 - 36;

    //desmos
<<<<<<< Updated upstream
    //https://www.desmos.com/calculator/taiu7i1jg8
    double a = 5.9;
    double b = 0.01864;
    double c = 0.177504;
    targetAngle = (a) * Math.sin(b*x) + c;
  


    System.out.println("encoder: " + leftArmEncoder.getPosition());
    System.out.println("degrees: " + motorRotationsToDegreesArm(leftArmEncoder.getPosition()));
    System.out.println("setpoint degrees: " + motorRotationsToDegreesArm(targetAngle) + ", x: " + x);
=======
    //https://www.desmos.com/calculator/y5sctj5btq
    // double a = 5.49648;
    // double b = 0.0148509;
    // double c = 0.482163;

    // double a = 8.8745;
    // double b = 0.0109488;
    // double c = -2.90105;

    double a = 10.0768;
    double b = 0.0119292;
    double c = -3.98546;
    targetAngle = (a) * Math.sin(b*(x)) + c; // old version
    System.out.println("x: " + x);

  


    // System.out.println(" encoder: " + leftArmEncoder.getPosition() + ", requested: " + targetAngle);
    // System.out.println(" degrees: " + motorRotationsToDegreesArm(leftArmEncoder.getPosition()));
    // System.out.println(" setpoint degrees: " + motorRotationsToDegreesArm(targetAngle) + ", x: " + x);
>>>>>>> Stashed changes
    
    if (ll_y == 0){
      return 0; //returns 0 bc LL val only 0 when nothing in sight (aka do nothing)
    }
    // targetAngle = 0;

    return targetAngle;
  }

  public double limelight_robot_rotation_aim_proportional() { // this function is good. Maybe could be faster if you want???
    double rotationTarget = 0;
    double dampen = 1; // use this to adjust how fast the thing goes (speed). Too high of a value will cause errors, so use pid too
    double rawRotationTarget = 0;
    // this function reads ll tx data and turns the robot for a speaker shot
    // read limelight tx
    // rotate robot until tx is ~~ 0

    if (Math.abs(ll_x) > 0.5) {
      rawRotationTarget = (ll_x) * (1) * dampen;
    }

    rotationTarget = map(rawRotationTarget, -28,  28, -3.14/2, 3.14/2); // dont tune this mapping function

    // pid values to tune
    double Kp = 0.025;
    double Ki = 0.01;
    double rotationTargetPID;
    rotationTargetPID = -PIDLoop(rawRotationTarget, Kp, Ki, rotationTarget); // don't think about this pid map too much. The real math doesn't work, but we only care about relative, so it is fine.

    // used to be == 0
    if (ll_x < 0.02) {
      return 0; //returns 0 bc LL val only 0 when nothing in sight (aka do nothing)
    }

    return rotationTargetPID;
  }

  // ____________________________________________________________________________
  
//original autospeaker

  public void autoSpeaker() {
    double target_area = .85; //.53
    double target_x = 0;
    double ll_deadzoneA = .01;
    double ll_deadzoneXY = 1.5; 
    //double target_y = 0;
    lastTime = m_timer.get();

    //control forward and back
    if (ll_area < (target_area - ll_deadzoneA) || ll_area > (target_area + ll_deadzoneA)) {
      double Kp = 0.85; //.81
      double Ki = 0.05; //.09
      commandY = PIDLoop(ll_area, Kp, Ki, target_area);
      // System.out.println("commandY: " + commandY);
      //commandY = command;
    } 

  //control left and right   

  if (ll_x < (target_x - ll_deadzoneXY) || ll_x > (target_x + ll_deadzoneXY)) {
       double Kp = .015;
       double Ki = .003;
       commandAngle = PIDLoop(ll_x, Kp, Ki, target_x);
      //  System.out.println("commandAngle: " + command);
       commandAngle = command;
     }

     //stop if can't see a tag
     if (ll_area <= 0.0002) {
       runningPID = false;
     }

     //drive and stop if command is too low
     if (Math.abs(commandAngle) < .1) {
       commandAngle = 0;
       //System.out.println("here");
     }

     //System.out.println("Command:" + commandY);
     if (Math.abs(commandY) < 0.07) {
        commandY = 0;
        //System.out.println("here2");
     }
    //  System.out.println("Total: " + (commandAngle + commandY));
     if(Math.abs(commandAngle) + Math.abs(commandY) < 0.11) {
        runningPID = false;
     }
    
    if(runningPID) {
      ChassisSpeeds csPID = new ChassisSpeeds(0, (-commandY*1.2), -commandAngle);
      swerveDrive.driveFieldOriented(csPID);
    }
    
    } 

//new autoSpeaker
/* 
 public void autoSpeaker() {
    double target_x = 0;
    double ll_deadzoneXY = 1.5; 
    //double target_y = 0;
    lastTime = m_timer.get();

    //Function to determine armControl from ll_area
        //I can send you the google doc with the equation from test data-we could probaly use some more test points


    //control Arm up and down 
    armControl(armGoal);

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
       //System.out.println("here");
     }

     //System.out.println("Command:" + commandY);
     if (Math.abs(commandY) < 0.07) {
        commandY = 0;
        //System.out.println("here2");
     }

     /*if(Math.abs(commandAngle + commandY) < 0.002) {
        teleShoot = true;
     }*/
/* 
     ChassisSpeeds csPID = new ChassisSpeeds(0, -commandY, -commandAngle);
     swerveDrive.driveFieldOriented(csPID);
  }*/



  public void autoAmp() {
    runningPIDAmp = true;
    double target_x = 0; 
    double ll_deadzone = 0;
    double commandX = 0;
    //control left and right 
     if (ll_x < (target_x - ll_deadzone) || ll_x > (target_x + ll_deadzone)) {
      double Kp = 0.05;//.020;
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
    
    maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    }
    catch(IOException i) {
   
    }

    t_timer.start();
    shooter.setInverted(false);

    gyro = (AHRS)swerveDrive.swerveDriveConfiguration.imu.getIMU();


    //NamedCommands.registerCommand("shoot", shootCommand());
    //NamedCommands.registerCommand("intake", new intakeCommand());
    
  }
 
  /* 
  public Command shootCommand() {
    autoShoot();
    return shootCommand();
  } */

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
<<<<<<< Updated upstream
    //- is right, + is left
=======
    /* 
>>>>>>> Stashed changes
    ChassisSpeeds cs_autoRight = new ChassisSpeeds(-0.35,0,0);
    ChassisSpeeds cs_autoLeft = new ChassisSpeeds(0.35,0,0);
    ChassisSpeeds cs_auto2 = new ChassisSpeeds(0,.35,0);
    ChassisSpeeds cs_autoForward = new ChassisSpeeds(0,-.35,0);
    ChassisSpeeds cs_autoRotateLeft = new ChassisSpeeds(0,0,-.2);
    ChassisSpeeds cs_autoRotateRight = new ChassisSpeeds(0,0,.2);

    ChassisSpeeds cs_fasterRotateLeft = new ChassisSpeeds(0,0,-0.55);
    ChassisSpeeds cs_fasterRotateRight = new ChassisSpeeds(0,0,0.55);

    NetworkTable table_b = NetworkTableInstance.getDefault().getTable("limelight-back");
    NetworkTable table_f = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTableEntry tx = table_b.getEntry("tx");
    NetworkTableEntry ty = table_b.getEntry("ty");
    NetworkTableEntry ta = table_b.getEntry("ta");

    NetworkTableEntry tx_f = table_f.getEntry("tx");
    NetworkTableEntry ty_f = table_f.getEntry("ty");
    NetworkTableEntry ta_f = table_f.getEntry("ta");

    //To get Limelight Data
    //read values periodically
    ll_x = tx.getDouble(0.0);
    ll_y = ty.getDouble(0.0);
    ll_area = ta.getDouble(0.0);

    //read values periodically
    ll_f_x = tx_f.getDouble(0.0);
    ll_f_y = ty_f.getDouble(0.0);
    ll_f_area = ta_f.getDouble(0.0);

    double lll_csX = 0;
    double lll_csY = 0;
    double lll_csAngle = 0;

<<<<<<< Updated upstream
=======
    double time = autoTimer.get();

    SmartDashboard.putNumber("timer", autoTimer.get());

    fourNoteFromCenterRed(time, lll_csX, lll_csY);



    //Original 4 ring auto, does not work with stage
    // double armSetpointAuto = 0;
   /*  
    if(0 < autoTimer.get() && autoTimer.get() < 1.3) {
      //shoot -- don't copy 
      teleShoot();
    }
    else if((1.3 < time) && (time < 2.2)) {
      //mmmmm note -- copy directly
      shooter.set(0);
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds(lll_csX, (-lll_csY * 1.2), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll); 
    }
    else if(2.2 < time && time < 2.5) {
      //extra intake time -- copy directly
      swerveDrive.drive(zeroSpeed);
    }
    else if(2.5 < time && time < 3) {
      //align -- copy directly
      armSetpoint = limelight_arm_aim_proportional() - 0.2;
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);
      // armSetpointAuto = limelight_arm_aim_proportional();
      // System.out.println("Arm setpoint: " + armSetpointAuto);
      // System.out.println("Current arm val: " + armEncoder);
      // double error2 = armSetpointAuto - armEncoder;
      // double armSetCommand = armSetPID.calculate(0, error2);
      // System.out.println("Command: " + armSetCommand);

      // if(armSetCommand > 0.4) {
      //   armSetCommand = 0.4;
      // }
      // if(armSetCommand < 0) {
      //   armSetCommand = 0;
      // }
      
      // //armControl(armSetpoint);
      // arm_left.set(armSetCommand);
      // arm_right.set(-armSetCommand);


      //System.out.println("enc: " + leftArmEncoder.getPosition());
      //System.out.println("encoder: " + leftArmEncoder.getPosition() + "degrees: " + motorRotationsToDegreesArm(leftArmEncoder.getPosition()) + "setpoint degrees: ");

    }
    else if(3 < time && time < 4.3) {
      //shoot - copy directly
      armControl(armSetpoint);
      
      swerveDrive.drive(zeroSpeed);
      autoShoot();
    }
    else if(4.3 < time && time < 5.2) {
      //turnTo
      shooter.set(0);
      ChassisSpeeds cs = new ChassisSpeeds(0,0,turnTo(90));
      swerveDrive.driveFieldOriented(cs);
    }
    else if(5.2 < autoTimer.get() && autoTimer.get() < 6.2) {
      //mmmmm note
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds(lll_csX, (-lll_csY * 1.2), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll);
    }
    else if(6.2 < autoTimer.get() && autoTimer.get() < 7) {
      //drive left and turnTo
      ChassisSpeeds cs2 = new ChassisSpeeds(-0.85, 0, turnTo(0));
      swerveDrive.driveFieldOriented(cs2);
    }
    else if(7 < autoTimer.get() && autoTimer.get() < 7.6) {
      //continue turnTo
      ChassisSpeeds cs3 = new ChassisSpeeds(0, 0, turnTo(0));
      swerveDrive.driveFieldOriented(cs3);
    }
    else if(7.6 < autoTimer.get() && autoTimer.get() < 8.1) {
      //align
      armSetpoint = limelight_arm_aim_proportional();
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);
    }
    else if(8.1 < time && time < 9.4) {
      //shoot
      armControl(armSetpoint);
      swerveDrive.drive(zeroSpeed);
      autoShoot();
    }
    else if(9.8 < time && time < 10.7) {
      //turnTo
      shooter.set(0);
      armSetpoint = 0;
      armControl(armSetpoint);
      ChassisSpeeds cs4 = new ChassisSpeeds(0,0,turnTo(270));
      swerveDrive.driveFieldOriented(cs4);
    }
    else if(10.7 < time && time < 11.9) {
      //mmmmm note
      shooter.set(0);
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds(lll_csX, (-lll_csY * 1.6), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll); 
    }
    else if(11.9 < time && time < 12.1) {
      //extra intake time
      intake.set(0.7);
      intake2.set(-0.7);
      swerveDrive.drive(zeroSpeed);
    }
    else if(12.1 < time && time < 12.7) {
      //drive right and start turnTo(0)
      //drive left and turnTo
      ChassisSpeeds cs2 = new ChassisSpeeds(1.3, 0, turnTo(315));
      swerveDrive.driveFieldOriented(cs2);
    }
    else if(12.7 < time && time < 13.2) {
      //finish turnTo(0)
      //continue turnTo
      ChassisSpeeds cs3 = new ChassisSpeeds(0, 0, turnTo(315));
      swerveDrive.driveFieldOriented(cs3);
    }
    else if(13.2 < time && time < 13.7) {
      //align
      armSetpoint = limelight_arm_aim_proportional();
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);
    }
    else {
      //shoot MAKE THE SHOT FINISH

      arm_left.set(0.037);
      arm_right.set(-0.037);
      swerveDrive.drive(zeroSpeed);
      autoShoot();
    }
    
    /* 
    else {
      shooter.set(0);
      feeder.set(0);
      intake.set(0);
      intake2.set(0);
      swerveDrive.drive(zeroSpeed);
    }
    */
    
     
    /* 
>>>>>>> Stashed changes
    if(0 < autoTimer.get() && autoTimer.get() < 1.3) {
      teleShoot();
    }
    if(1.3 < autoTimer.get() && autoTimer.get() < 2.3) {
      shooter.set(0);
      feeder.set(0.5);
      intake.set(0.5);
      intake2.set(-0.5);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      if (ll_f_y >= -40) {
        lll_csY = -0.5;}
      }
      ChassisSpeeds lll = new ChassisSpeeds(lll_csX, (-lll_csY * 1.2), lll_csAngle);
      swerveDrive.drive(lll); 
    }
    if(2.3 < autoTimer.get() && autoTimer.get() < 3.4) {
      //feeder.set(0.2);
      runningPID = true;
      autoSpeaker();
      //align

    }
    if(3.4 < autoTimer.get() && autoTimer.get() < 4.7) {
      runningPID = false;
      swerveDrive.driveFieldOriented(zeroSpeed);
      teleShoot();
      //shoot
    }
    if(4.7 < autoTimer.get() && autoTimer.get() < 5.2) {
      swerveDrive.driveFieldOriented(cs_fasterRotateRight);
    }
    if(5.2 < autoTimer.get() && autoTimer.get() < 6.8) {
      if(!hasReached4) {
        swerveDrive.drive(zeroSpeed);
        hasReached4 = true;
      }
      shooter.set(0);
      feeder.set(0.5);
      intake.set(0.5);
      intake2.set(-0.5);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      if (ll_f_y >= -40) {
        lll_csY = -0.5;}
      }
      ChassisSpeeds lll = new ChassisSpeeds(lll_csX, (-lll_csY * 1.2), lll_csAngle);
      swerveDrive.drive(lll);
    }
    if(8 < autoTimer.get() && autoTimer.get() < 8.5) {
      //rotate right and move right
    }
    if(8.5 < autoTimer.get() && autoTimer.get() < 9.6) {
      //align
    }
    if(9.6 < autoTimer.get() && autoTimer.get() < 10.9) {
      //shoot
    }
    if(10.9 < autoTimer.get() && autoTimer.get() < 11.4) {
      //rotate left
    }
    if(11.4 < autoTimer.get() && autoTimer.get() < 12.6) {
      //intake and mmmmm ring
    }
    if(12.6 < autoTimer.get() && autoTimer.get() < 13.1) {
      //rotate and move right
    }
<<<<<<< Updated upstream
    if(13.1 < autoTimer.get() && autoTimer.get() < 13.6)
=======
    if(13.1 < autoTimer.get() && autoTimer.get() < 13.6) {

    }
    */
//auto V2 shooting from center -- TUNED FOR 12.7 - 12.8V
    
    
    /* 
    else {
      shooter.set(0);
      feeder.set(0);
      intake.set(0);
      intake2.set(0);
      swerveDrive.drive(zeroSpeed);
    }
    */
    
     
    /* 
    if(0 < autoTimer.get() && autoTimer.get() < 1.3) {
      teleShoot();
    }
    if(1.3 < autoTimer.get() && autoTimer.get() < 2.3) {
      shooter.set(0);
      feeder.set(0.5);
      intake.set(0.5);
      intake2.set(-0.5);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      if (ll_f_y >= -40) {
        lll_csY = -0.5;}
      }
      ChassisSpeeds lll = new ChassisSpeeds(lll_csX, (-lll_csY * 1.2), lll_csAngle);
      swerveDrive.drive(lll); 
    }
    if(2.3 < autoTimer.get() && autoTimer.get() < 3.4) {
      //feeder.set(0.2);
      runningPID = true;
      autoSpeaker();
      //align

    }
    if(3.4 < autoTimer.get() && autoTimer.get() < 4.7) {
      runningPID = false;
      swerveDrive.driveFieldOriented(zeroSpeed);
      teleShoot();
      //shoot
    }
    if(4.7 < autoTimer.get() && autoTimer.get() < 5.2) {
      swerveDrive.driveFieldOriented(cs_fasterRotateRight);
    }
    if(5.2 < autoTimer.get() && autoTimer.get() < 6.8) {
      if(!hasReached4) {
        swerveDrive.drive(zeroSpeed);
        hasReached4 = true;
      }
      shooter.set(0);
      feeder.set(0.5);
      intake.set(0.5);
      intake2.set(-0.5);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      if (ll_f_y >= -40) {
        lll_csY = -0.5;}
      }
      ChassisSpeeds lll = new ChassisSpeeds(lll_csX, (-lll_csY * 1.2), lll_csAngle);
      swerveDrive.drive(lll);
    }
    if(8 < autoTimer.get() && autoTimer.get() < 8.5) {
      //rotate right and move right
    }
    if(8.5 < autoTimer.get() && autoTimer.get() < 9.6) {
      //align
    }
    if(9.6 < autoTimer.get() && autoTimer.get() < 10.9) {
      //shoot
    }
    if(10.9 < autoTimer.get() && autoTimer.get() < 11.4) {
      //rotate left
    }
    if(11.4 < autoTimer.get() && autoTimer.get() < 12.6) {
      //intake and mmmmm ring
    }
    if(12.6 < autoTimer.get() && autoTimer.get() < 13.1) {
      //rotate and move right
    }
    if(13.1 < autoTimer.get() && autoTimer.get() < 13.6) {

    }

>>>>>>> Stashed changes

    /*Auto Options:
     * 1:Shoot, Drive back Left  *****
     * 1.5: shoot, drive back, right  ******
     * 2:Shoot, drive back, pick up another ring
     * 3:Shoot, drive back, pick up ring, shoot again *****
     * 4:Drive back
     * 5:DO NOT USE THIS
     */

    // System.out.println(autoTimer.get());
    //Auto option 1: Shoot, drive back, left
    /*if (autoTimer.get() < 3.1){
    
    } 
    else if(autoTimer.get() < 6.1){
        autoShoot();
    } else if (autoTimer.get() < 6.6){
       swerveDrive.driveFieldOriented(cs_autoLeft);
       feeder.set(0);
       shooter.set(0);
    } else if (autoTimer.get() < 10.1){
       swerveDrive.driveFieldOriented(cs_auto2);
    } else {
      swerveDrive.driveFieldOriented(zeroSpeed);
    }
*/

    //Auto option 1.5: Shoot, drive back right
    /* 
    if (autoTimer.get() < 3.1){
      
    } 
    else if(autoTimer.get() < 6.1){
        autoShoot();
    } else if (autoTimer.get() < 8.5){
       swerveDrive.driveFieldOriented(cs_autoRight);
       feeder.set(0);
       shooter.set(0);
    } else if (autoTimer.get() < 12){
       swerveDrive.driveFieldOriented(cs_auto2);
    } else {
      swerveDrive.driveFieldOriented(zeroSpeed);
    }
    */
    

     

   /* //Auto option 2: Shoot, drive back, pick up another ring
      if (autoTimer.get() < 3){
        autoShoot();
      } else if (autoTimer.get() < 6){
        swerveDrive.driveFieldOriented(cs_auto2);
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

     //Auto option 3: Shoot, drive back, pick up ring, shoot again from center
    /* 
     if (autoTimer.get() < 3){
        autoShoot();
      } else if (autoTimer.get() < 5){
        swerveDrive.driveFieldOriented(cs_auto2);
        shooter.set(0);
        intake.set(0.5);
        intake2.set(-0.5);
        feeder.set(.5);
      } else if (autoTimer.get() < 5.3){
        intake.set(0.5);
        intake2.set(-0.5);
        feeder.set(.5);
        shooter.set(0); 
      } else if (autoTimer.get() < 6.5) {  
        swerveDrive.driveFieldOriented(zeroSpeed);
      } else if (autoTimer.get() < 6.8){
        intake.set(0);
        intake2.set(0);
        feeder.set(0);
      } else if (autoTimer.get() < 9) {
        swerveDrive.driveFieldOriented(cs_autoForward);
      } else if (autoTimer.get() < 13) {
        if(!hasReached2) {
          hasReached = false;
          hasReached2 = true;
        } 
        autoShoot();
        swerveDrive.driveFieldOriented(zeroSpeed);
      } else {
        shooter.set(0); 
      }
      */
       
      
      
     

    //Auto option 4: Drive
    /*
    if (autoTimer.get() < 3){
       swerveDrive.driveFieldOriented(cs_auto);
    } else {
      swerveDrive.driveFieldOriented(zeroSpeed);
    }*/
 
    //5 doesn't work
 //Auto option 5: Shoot, drive back, pick up ring, shoot again from right side
    /* 
     if (autoTimer.get() < 3){
        autoShoot();
      } else if (autoTimer.get() < 5){
        swerveDrive.driveFieldOriented(cs_auto2);
        intake.set(0.25);
        intake2.set(-0.25);
        feeder.set(.25);
        shooter.set(0); 
      } else if (autoTimer.get() < 6.5) {  
        swerveDrive.driveFieldOriented(zeroSpeed);
      } else if (autoTimer.get() < 6.8){
        intake.set(0);
        intake2.set(0);
        feeder.set(0);
      } else if (autoTimer.get() < 8.8) {
        swerveDrive.driveFieldOriented(cs_autoForward);
      } else if (autoTimer.get() < 13) {
        if(!hasReached2) {
          hasReached = false;
          hasReached2 = true;
        } 
        autoShoot();
        swerveDrive.driveFieldOriented(zeroSpeed);
      } else {
        shooter.set(0); 
      }
*/     
  }


  //changed a few things in code
    //final shoot code
    //time before second shot 

  public void fourNoteFromCenterRed(double time, double lll_csX, double lll_csY) {
    if(0 < autoTimer.get() && autoTimer.get() < 1.3) {
      //shoot -- don't copy 
      teleShoot();
    }
    else if((1.3 < time) && (time < 2.2)) {
      //mmmmm note -- copy directly
      shooter.set(0);
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds(lll_csX, (-lll_csY * 1.2), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll); 
    }
    else if(2.2 < time && time < 2.5) {
      //extra intake time -- copy directly
      swerveDrive.drive(zeroSpeed);
    }
    else if(2.5 < autoTimer.get() && autoTimer.get() < 2.9) {
     //drive forward
      ChassisSpeeds csForward = new ChassisSpeeds(0, -1.2, 0);
      swerveDrive.driveFieldOriented(csForward);
    }
    else if(2.9 < time && time < 3.3) {
      //align -- copy directly
      armSetpoint = limelight_arm_aim_proportional() - 0.2;
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);

    }
    else if (3.3 < time && time < 3.4) {
      swerveDrive.drive(zeroSpeed);
    }
    else if(3.4 < time && time < 4.6) {
      //shoot - copy directly
      arm_left.set(0.037);
      arm_right.set(-0.037);
      autoShoot();
    }
    else if(4.6 < time && time < 5.3) {
      //turnTo
      armSetpoint = 0;
      armControl(0);
      shooter.set(0);
      ChassisSpeeds cs = new ChassisSpeeds(0,0,turnTo(55));
      swerveDrive.driveFieldOriented(cs);
    }
    else if(5.3 < time && time < 6.5) {
      //mmmmm note
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds((lll_csX * 1.2), (-lll_csY * 1.2), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll);
    }
    else if(6.5 < time && time < 7.3) {
      //drive back to speaker and turnTo
      ChassisSpeeds cs2 = new ChassisSpeeds(-1.15, -0.65, turnTo(0));
      swerveDrive.driveFieldOriented(cs2);
    }
    else if(7.3 < autoTimer.get() && autoTimer.get() < 8) {
      //finish turnTo
      ChassisSpeeds cs3 = new ChassisSpeeds(0, 0, turnTo(0));
      swerveDrive.driveFieldOriented(cs3);
    }
    else if(8 < autoTimer.get() && autoTimer.get() < 8.4) {
      //align
      armSetpoint = limelight_arm_aim_proportional();
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);
    }
    else if(8.4 < time && time < 9.7) {
      //shoot
      armControl(armSetpoint);
      swerveDrive.drive(zeroSpeed);
      autoShoot();
    }
    else if(9.7 < time && time < 10.8) {
      //turnTo
      shooter.set(0);
      armSetpoint = 0;
      armControl(armSetpoint);
      ChassisSpeeds cs4 = new ChassisSpeeds(0,0,turnTo(315));
      swerveDrive.driveFieldOriented(cs4);
    }
    else if(10.8 < time && time < 12.2) {
      //mmmmm note
      shooter.set(0);
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds((lll_csX * 1.15), (-lll_csY * 1.0), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll); 
    }
    else if(12.2 < time && time < 12.4) {
      //extra intake time
      intake.set(0.7);
      intake2.set(-0.7);
      shooter.set(0);
      swerveDrive.drive(zeroSpeed);
    }
    else if(12.4 < time && time < 12.9) {
      //drive
      ChassisSpeeds cs5 = new ChassisSpeeds(1.15, 0, turnTo(0));
      swerveDrive.driveFieldOriented(cs5);
    }
    else if(12.9 < time && time < 13.3) {
      ChassisSpeeds cs6 = new ChassisSpeeds(0, -0.95, turnTo(0));
      swerveDrive.driveFieldOriented(cs6);
    }
    else if(13.3 < time && time < 13.7) {
      //align
      armSetpoint = limelight_arm_aim_proportional();
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);
    }
    else if (13.7 < time && 13.8 < time) {
      //shoot MAKE THE SHOT FINISH
      arm_left.set(0.037);
      arm_right.set(-0.037);
      swerveDrive.drive(zeroSpeed);
    }
    else if (13.8 < time && 15 < time) {
      autoShoot();
    }
    else  {

    }
  }

  public void fourNoteFromCenterBlue(double time, double lll_csX, double lll_csY) {
    if(0 < autoTimer.get() && autoTimer.get() < 1.3) {
      //shoot -- don't copy 
      teleShoot();
    }
    else if((1.3 < time) && (time < 2.2)) {
      //mmmmm note -- copy directly
      shooter.set(0);
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds(lll_csX, (-lll_csY * 1.2), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll); 
    }
    else if(2.2 < time && time < 2.5) {
      //extra intake time -- copy directly
      swerveDrive.drive(zeroSpeed);
    }
    else if(2.5 < autoTimer.get() && autoTimer.get() < 2.9) {
     //drive forward
      ChassisSpeeds csForward = new ChassisSpeeds(0, -1.2, 0);
      swerveDrive.driveFieldOriented(csForward);
    }
    else if(2.9 < time && time < 3.3) {
      //align -- copy directly
      armSetpoint = limelight_arm_aim_proportional() - 0.2;
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);

    }
    else if (3.3 < time && time < 3.4) {
      swerveDrive.drive(zeroSpeed);
    }
    else if(3.4 < time && time < 4.6) {
      //shoot - copy directly
      arm_left.set(0.037);
      arm_right.set(-0.037);
      swerveDrive.drive(zeroSpeed);
      autoShoot();
    }
    else if(4.6 < time && time < 5.3) {
      //turnTo
      armSetpoint = 0;
      armControl(0);
      shooter.set(0);
      ChassisSpeeds cs = new ChassisSpeeds(0,0,turnTo(305));
      swerveDrive.driveFieldOriented(cs);
    }
    else if(5.3 < time && time < 6.5) {
      //mmmmm note
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds((lll_csX * 1.2), (-lll_csY * 1.2), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll);
    }
    else if(6.5 < time && time < 7.3) {
      //drive back to speaker and turnTo
      ChassisSpeeds cs2 = new ChassisSpeeds(1.15, -0.65, turnTo(0));
      swerveDrive.driveFieldOriented(cs2);
    }
    else if(7.3 < autoTimer.get() && autoTimer.get() < 8) {
      //finish turnTo
      ChassisSpeeds cs3 = new ChassisSpeeds(0, 0, turnTo(0));
      swerveDrive.driveFieldOriented(cs3);
    }
    else if(8 < autoTimer.get() && autoTimer.get() < 8.4) {
      //align
      armSetpoint = limelight_arm_aim_proportional();
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);
    }
    else if(8.4 < time && time < 9.7) {
      //shoot
      armControl(armSetpoint);
      swerveDrive.drive(zeroSpeed);
      autoShoot();
    }
    else if(9.7 < time && time < 10.8) {
      //turnTo
      shooter.set(0);
      armSetpoint = 0;
      armControl(armSetpoint);
      ChassisSpeeds cs4 = new ChassisSpeeds(0,0,turnTo(45));
      swerveDrive.driveFieldOriented(cs4);
    }
    else if(10.8 < time && time < 12.2) {
      //mmmmm note
      shooter.set(0);
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds((lll_csX * 1.15), (-lll_csY * 1.0), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll); 
    }
    else if(12.2 < time && time < 12.4) {
      //extra intake time
      // intake.set(0.7);
      // intake2.set(-0.7);
      shooter.set(0);
      swerveDrive.drive(zeroSpeed);
    }
    else if(12.4 < time && time < 12.9) {
      //drive
      ChassisSpeeds cs5 = new ChassisSpeeds(-1.15, 0, turnTo(0));
      swerveDrive.driveFieldOriented(cs5);
    }
    else if(12.9 < time && time < 13.3) {
      ChassisSpeeds cs6 = new ChassisSpeeds(0, -0.95, turnTo(0));
      swerveDrive.driveFieldOriented(cs6);
    }
    else if(13.3 < time && time < 13.7) {
      //align
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);

      armSetpoint = limelight_arm_aim_proportional();
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
    }else if (13.7 < time && 13.8 < time) {
      //shoot MAKE THE SHOT FINISH
      arm_left.set(0.037);
      arm_right.set(-0.037);
      swerveDrive.drive(zeroSpeed);
    }
    else if (13.8 < time && 15 < time) {
      autoShoot();
    }
    else  {
      
    }
  }

   public void fourNoteFromCenterRedSpedUp(double time, double lll_csX, double lll_csY) {
    if(0 < autoTimer.get() && autoTimer.get() < 1.3) {
      //shoot -- don't copy 
      teleShoot();
    }
    else if((1.3 < time) && (time < 2.2)) {
      //mmmmm note -- copy directly
      shooter.set(0);
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds(lll_csX, (-lll_csY * 1.2), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll); 
    }
    else if(2.2 < time && time < 2.5) {
      //extra intake time -- copy directly
      swerveDrive.drive(zeroSpeed);
    }
    else if(2.5 < autoTimer.get() && autoTimer.get() < 2.9) {
     //drive forward
      ChassisSpeeds csForward = new ChassisSpeeds(0, -1.2, 0);
      swerveDrive.driveFieldOriented(csForward);
    }
    else if(2.9 < time && time < 3.3) {
      //align -- copy directly
      armSetpoint = limelight_arm_aim_proportional() - 0.2;
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);

    }
    else if (3.3 < time && time < 3.4) {
      swerveDrive.drive(zeroSpeed);
    }
    else if(3.4 < time && time < 4.6) {
      //shoot - copy directly
      arm_left.set(0.037);
      arm_right.set(-0.037);
      autoShoot();
    }
    else if(4.6 < time && time < 5.3) {
      //turnTo
      armSetpoint = 0;
      armControl(0);
      shooter.set(0);
      ChassisSpeeds cs = new ChassisSpeeds(0,0,turnTo(55));
      swerveDrive.driveFieldOriented(cs);
    }
    else if(5.3 < time && time < 6.5) {
      //mmmmm note
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds((lll_csX * 1.2), (-lll_csY * 1.2), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll);
    }
    else if(6.5 < time && time < 7.3) {
      //drive back to speaker and turnTo
      ChassisSpeeds cs2 = new ChassisSpeeds(-1.15, -0.65, turnTo(0));
      swerveDrive.driveFieldOriented(cs2);
    } //took extra time from statement below
    else if(7.3 < autoTimer.get() && autoTimer.get() < 7.5) {
      //finish turnTo
      ChassisSpeeds cs3 = new ChassisSpeeds(0, 0, turnTo(0));
      swerveDrive.driveFieldOriented(cs3);
    }
    else if(7.5 < autoTimer.get() && autoTimer.get() < 7.9) {
      //align
      armSetpoint = limelight_arm_aim_proportional();
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);
    }
    else if(7.9 < time && time < 9.2) {
      //shoot
      armControl(armSetpoint);
      swerveDrive.drive(zeroSpeed);
      autoShoot();
    }
    else if(9.2 < time && time < 10.3) {
      //turnTo
      shooter.set(0);
      armSetpoint = 0;
      armControl(armSetpoint);
      ChassisSpeeds cs4 = new ChassisSpeeds(0,0,turnTo(315));
      swerveDrive.driveFieldOriented(cs4);
    }
    else if(10.3 < time && time < 11.7) {
      //mmmmm note
      shooter.set(0);
      feeder.set(0.7);
      intake.set(0.7);
      intake2.set(-0.7);

      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      }
      if (ll_f_y >= -40) {
        lll_csY = -0.5;
      }
      ChassisSpeeds lll = new ChassisSpeeds((lll_csX * 1.15), (-lll_csY * 1.0), 0);
      SmartDashboard.putNumber("lll_csX", lll_csX);
      SmartDashboard.putNumber("lll_csY", lll_csY);
      swerveDrive.drive(lll); 
    }
    else if(12.2 < time && time < 12.4) {
      //extra intake time
      intake.set(0.7);
      intake2.set(-0.7);
      shooter.set(0);
      swerveDrive.drive(zeroSpeed);
    }
    else if(11.7 < time && time < 12.4) {
      //drive
      ChassisSpeeds cs5 = new ChassisSpeeds(1.15, 0, turnTo(0));
      swerveDrive.driveFieldOriented(cs5);
    }
    else if(12.4 < time && time < 12.8) {
      ChassisSpeeds cs6 = new ChassisSpeeds(0, -0.95, turnTo(0));
      swerveDrive.driveFieldOriented(cs6);
    }
    else if(12.8 < time && time < 13.2) {
      //align
      armSetpoint = limelight_arm_aim_proportional();
      armControl(armSetpoint);

      double robotRotation = limelight_robot_rotation_aim_proportional();
      System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);


      swerveDrive.drive(llFront);  
      
      intake.set(0);
      intake2.set(0);
      feeder.set(0);
      shooter.set(0);
    }
    else if (13.2 < time && 13.3 < time) {
      //shoot MAKE THE SHOT FINISH
      arm_left.set(0.037);
      arm_right.set(-0.037);
      swerveDrive.drive(zeroSpeed);
    }
    else if (13.3 < time && 15 < time) {
      autoShoot();
    }
    else  {

    }
  }



/* 
  public void updateOdometry() {
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });



    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if(limelightMeasurement.tagCount >= 2)
    {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      m_poseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds);
    }
  }
  */

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
    shooter.set(0);
    feeder.set(0);

    gyro.zeroYaw();
    // swerveDrive.
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

<<<<<<< Updated upstream
=======
    // double org = swerveDrive.getYaw().getDegrees();
    // double org2 = gyro.getYaw();

    // org = (-org);

    // System.out.println("Direct yaw: " + org);
    // System.out.println("Indirect yaw: " + org2);
    //System.out.println("armval: " + leftArmEncoder.getPosition() + ", degrees: " + motorRotationsToDegreesArm(leftArmEncoder.getPosition()));

>>>>>>> Stashed changes
    // System.out.println("encoder " + leftArmEncoder.getPosition());
   
    NetworkTable table_b = NetworkTableInstance.getDefault().getTable("limelight-back");
    NetworkTable table_f = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTableEntry tx = table_b.getEntry("tx");
    NetworkTableEntry ty = table_b.getEntry("ty");
    NetworkTableEntry ta = table_b.getEntry("ta");

    NetworkTableEntry tx_f = table_f.getEntry("tx");
    NetworkTableEntry ty_f = table_f.getEntry("ty");
    NetworkTableEntry ta_f = table_f.getEntry("ta");

    // System.out.println(leftArmEncoder.getPosition());

    //To get Limelight Data
    //read values periodically
    ll_x = tx.getDouble(0.0);
    ll_y = ty.getDouble(0.0);
    ll_area = ta.getDouble(0.0);

    //read values periodically
    ll_f_x = tx_f.getDouble(0.0);
    ll_f_y = ty_f.getDouble(0.0);
    ll_f_area = ta_f.getDouble(0.0);
    //table_b.getEntry("pipeline").setValue(0);
     //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", ll_x);
    SmartDashboard.putNumber("LimelightY", ll_y);
    SmartDashboard.putNumber("LimelightArea", ll_area);
    
//  System.out.println("limelight " + ll_area);
//  System.out.println("LLX"+ll_x);
//  System.out.println("LLY"+ll_y);

    
    //System.out.println("Area: " + ll_area + " Angle: " + ll_x);
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
      intake.set(0.8); //prev 0.5 before current limited to 20A
      intake2.set(-0.8);
      feeder.set(.8);
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

    //Pick a button for this 
  /* if (controller.get()) {
      turnTo(90);
    } */

    /*if ((controller2.getRightTriggerAxis() > 0.3) || controller.getAButton()) {
      autoShoot();
      //I would like to have autoshoot be our only shooting command, but it isnt wokring right now
    }*/ //I think the feeder is being turned off somewhere in the main loop

     if ((controller2.getRightTriggerAxis() > 0.3)) {
      armSetpoint = 3;
      //armControl(armSetpoint);
      //feeder.set(.5);
      //I would like to have autoshoot be our only shooting command, but it isnt wokring right now
    }
     
    if(controller2.getRightBumper()) {
      feeder.set(-.1); 
      shooterProxy = 0.1; // now positive with gear. This operation doesn't work properly
    }
    if(controller.getAButton()) {
      //shooter.set(-0.6);
      runningPID = true;
      autoSpeaker();
    } else {
      //shooterProxy = 0;
      runningPID = false;
    }

    if(controller2.getRightStickButton()) {
      teleShoot();
      System.out.println("right stick down");
    }
    
    if(!controller2.getRightBumper() && !controller.getAButton() && !controller2.getRightStickButton()) {
      shooterProxy = 0;
    }
    if(!controller2.getRightStickButton()) {
      shooter.set(shooterProxy);
    }

    // "mmmm note" limelight note homing by jake :)
    // limelight note test thing. homing
    double lll_csX = 0;
    double lll_csY = 0;
    double lll_csAngle = 0;
    if (controller2.getLeftStickButton()) {
      
      System.out.println("mmmmm note " + ll_f_x +" "+ ll_f_y);
      if (Math.abs(ll_f_x) >= 0.1) {
        lll_csX = 0.01 * ll_f_x;
      if (ll_f_y >= -40) {
        lll_csY = -0.5;}
      }
      ChassisSpeeds lll = new ChassisSpeeds((lll_csX*1.5), -(lll_csY*1.2), lll_csAngle);
      swerveDrive.drive(lll);  
    }

    // jake pt2
    // limelight 3 aiming by jake :) pt 2
    else if (controller2.getAButton()) {
      armSetpoint = limelight_arm_aim_proportional();
      armControl(armSetpoint);
      // System.out.println("enc: " + leftArmEncoder.getPosition());
      // System.out.println("encoder: " + leftArmEncoder.getPosition() + "degrees: " + motorRotationsToDegreesArm(leftArmEncoder.getPosition()) + "setpoint degrees: ");


      double robotRotation = limelight_robot_rotation_aim_proportional();
      // System.out.println(robotRotation);
      ChassisSpeeds llFront = new ChassisSpeeds(0, 0, robotRotation);
      swerveDrive.drive(llFront);  
    }

    else {
      if(!runningPID && !runningPIDAmp) {
      ChassisSpeeds cs = new ChassisSpeeds(csX,-csY,csAngle);
      
      swerveDrive.driveFieldOriented(cs);
     }
    }
    
    //Shuffled around the axes to make the actual front the front.
  
    //System.out.println(SwerveMath.calculateMetersPerRotation(0.1, 6.75, 1));
    //System.out.println(SwerveMath.calculateDegreesPerSteeringRotation(21.428, 1));

  if (controller.getLeftTriggerAxis() > .3) {
    climberLeft.set(.27);
    climberRight.set(-.27);
  } else if (controller.getRightTriggerAxis() > .3){
    climberLeft.set(-.27);
    climberRight.set(.27);
  } else {
    climberLeft.set(0);
    climberRight.set(0);
  }

  //If you want to run the arm in any other loop, se armPresetRunning to true

    if (controller.getBButtonPressed() ){//|| controller2.getAButtonPressed()) {
        //0 arm
        armSetpoint = 0;
        armPresetRunning = true;
    }

     if (controller.getYButtonPressed()) {
      armSetpoint = 31;
      armPresetRunning = true;
    // runningPIDAmp = true;  
   }


    if (controller2.getYButton()) {
        armPresetRunning = false;
        arm_left.set(0.1);
        arm_right.set(-0.1);
        armSetpoint = leftArmEncoder.getPosition();
    } else if (controller2.getBButton()) {
        armPresetRunning = false;
        arm_left.set(-0.1);
        arm_right.set(0.1);
        armSetpoint = leftArmEncoder.getPosition();
    }else {
      armControl(armSetpoint);
    }

   //reverse intake
   
    if (controller2.getLeftTriggerAxis() > .3){
      intake.set(-.1);
      intake2.set(.1);
    }
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

  
  
    

  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
<<<<<<< Updated upstream
    CommandScheduler.getInstance().cancelAll();
=======
    
>>>>>>> Stashed changes
  }

  /** This function is called periodically during test mode. */
  @Override
<<<<<<< Updated upstream
  public void testPeriodic() {}
=======
  public void testPeriodic() {
    /*ChassisSpeeds cs = new ChassisSpeeds(0,0,turnTo(180));
    swerveDrive.driveFieldOriented(cs);
    System.out.println(swerveDrive.getYaw());*/

      ChassisSpeeds csForward = new ChassisSpeeds(0, 0, turnTo(305));
      swerveDrive.driveFieldOriented(csForward);
  }
>>>>>>> Stashed changes

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

//Path Planner Auto Code
//If things break try commenting everything under here out other than the bracket


public class SwerveSubsystem extends SubsystemBase
{
  
 /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    setupPathPlanner();
  }

  
  /**
   * Setup AutoBuilder for PathPlanner.
   */

  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
                                                            );
  }
  
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose); 
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }
}

//Auto 
//All written in command structures

/*
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("a");
   // return autoChooser.getSelected();
  }
  */



//Shoot

//drive

//intake
}




  






