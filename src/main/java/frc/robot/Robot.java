// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.encoders.CANCoderSwerve;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private SwerveDrive swerveDrive;
  private final XboxController controller = new XboxController(0);
  private boolean turtleMode = false;


  // CANCoderSwerve1 = new CANCoderSwerve(34);
  // CANCoderSwerve2 = new CANCoderSwerve();
  // CANCoderSwerve3 = new CANCoderSwerve();
  // CANCoderSwerve4 = new CANCoderSwerve();

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
  public void autonomousPeriodic() {}

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

    if(controller.getYButton()) {
      if(turtleMode) {
        turtleMode = false;
      }
      else {
        turtleMode = true;
      }
    }


    double movementDeadzone = 0.3;
    //0.6,1
    double maxSpeedX = 1.5, maxSpeedY = 1.5, maxSpeedAngle = 1.5;
    double csX = 0, csY = 0, csAngle = 0;
    if(turtleMode) {
      maxSpeedX = 0.2;
      maxSpeedY = 0.2;
      maxSpeedAngle = 0.5;
    }
    //X an Y are reversed and it is CORRECT! it make field oriented drive work.
    if(controller.getLeftX() > movementDeadzone) {
      csX = controller.getLeftX() * maxSpeedY;
    }
    else if(controller.getLeftX() < -movementDeadzone) {
      csX = controller.getLeftX() * maxSpeedY;
    }

    if(controller.getLeftY() > movementDeadzone) {
      csY = controller.getLeftY() * maxSpeedX;
    }
    else if(controller.getLeftY() < -movementDeadzone) {
      csY = controller.getLeftY() * maxSpeedX;
    }

    if(controller.getRightX() > movementDeadzone) {
      csAngle = controller.getRightX() * maxSpeedAngle;
    }
    else if(controller.getRightX() < -movementDeadzone) {
      csAngle = controller.getRightX() * maxSpeedAngle;
    }
    
    //Shuffled around the axes to make the actual front the front.
    ChassisSpeeds cs = new ChassisSpeeds(csY,csX,csAngle);
    swerveDrive.driveFieldOriented(cs);
  
    //System.out.println(SwerveMath.calculateMetersPerRotation(0.1, 6.75, 1));
    //System.out.println(SwerveMath.calculateDegreesPerSteeringRotation(21.428, 1));
    
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
