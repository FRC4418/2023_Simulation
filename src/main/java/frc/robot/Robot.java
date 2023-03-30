// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private AutoGamepad driver = new AutoGamepad(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double throttle = driver.getLeftX();
    double turn = driver.getLeftY();

    m_robotContainer.driveSubsystem.leftFrontMotor.set(ControlMode.PercentOutput, throttle, DemandType.ArbitraryFeedForward, turn);
    m_robotContainer.driveSubsystem.rightFrontMotor.set(ControlMode.PercentOutput, throttle, DemandType.ArbitraryFeedForward, -turn);
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
  public void simulationPeriodic() {
    m_robotContainer.driveSubsystem.leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_robotContainer.driveSubsystem.rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_robotContainer.driveSubsystem.m_drivetrainSimulator.setInputs(
      m_robotContainer.driveSubsystem.leftDriveSim.getMotorOutputLeadVoltage(), 
      m_robotContainer.driveSubsystem.rightDriveSim.getMotorOutputLeadVoltage());

      m_robotContainer.driveSubsystem.m_drivetrainSimulator.update(0.02);

      m_robotContainer.driveSubsystem.leftDriveSim.setIntegratedSensorRawPosition((int)m_robotContainer.driveSubsystem.nativeUnitsToDistanceMeters(m_robotContainer.driveSubsystem.m_drivetrainSimulator.getLeftPositionMeters()));
      m_robotContainer.driveSubsystem.rightDriveSim.setIntegratedSensorRawPosition((int)m_robotContainer.driveSubsystem.nativeUnitsToDistanceMeters(m_robotContainer.driveSubsystem.m_drivetrainSimulator.getRightPositionMeters()));
      
      m_robotContainer.driveSubsystem.leftDriveSim.setIntegratedSensorVelocity((int)m_robotContainer.driveSubsystem.velocityToNativeUnits(m_robotContainer.driveSubsystem.m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
      m_robotContainer.driveSubsystem.rightDriveSim.setIntegratedSensorVelocity((int)m_robotContainer.driveSubsystem.velocityToNativeUnits(m_robotContainer.driveSubsystem.m_drivetrainSimulator.getRightVelocityMetersPerSecond()));
      
      m_robotContainer.driveSubsystem.gyroSim.setAngle(m_robotContainer.driveSubsystem.m_drivetrainSimulator.getHeading().getDegrees());
    }
}
