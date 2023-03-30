// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class driveTrain extends SubsystemBase {
  /** Creates a new driveTrain. */
  public final WPI_TalonFX leftFrontMotor = new WPI_TalonFX(4);
  public final WPI_TalonFX leftBackMotor = new WPI_TalonFX(1);
  public MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);

  public final WPI_TalonFX rightFrontMotor = new WPI_TalonFX(3);
  public final WPI_TalonFX rightBackMotor = new WPI_TalonFX(2);
  public MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  private final AnalogGyro gyro = new AnalogGyro(0);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.driveSettings.kTrackWidth);

  public final AnalogGyroSim gyroSim = new AnalogGyroSim(0);

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(), 
    0,0);

  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.14971, 0.0073732);

  private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

  public TalonFXSimCollection leftDriveSim = leftFrontMotor.getSimCollection();
  public TalonFXSimCollection rightDriveSim = rightFrontMotor.getSimCollection();

  private final Field2d field = new Field2d();
  
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
  LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);

  //private final DifferentialDrivetrainSim driveTrainSim = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2), 7.75, 3.0, 31.75, 0.0762, VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  public final DifferentialDrivetrainSim m_drivetrainSimulator =
  new DifferentialDrivetrainSim(
      m_drivetrainSystem, 
      DCMotor.getFalcon500(2), 
      7.75, 
      3.0, 
      31.75, 
      null);

  public driveTrain() {
    leftFrontMotor.configFactoryDefault();
    leftBackMotor.configFactoryDefault();
    leftBackMotor.follow(leftFrontMotor);
    leftBackMotor.setInverted(InvertType.FollowMaster);

    rightFrontMotor.configFactoryDefault();
    rightBackMotor.configFactoryDefault();
    rightBackMotor.follow(rightFrontMotor);
    rightBackMotor.setInverted(InvertType.FollowMaster);

    SmartDashboard.putData("field", field);
    
    rightFrontMotor.setInverted(InvertType.InvertMotorOutput);


  }
  public int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(3.0));
    double motorRotations = wheelRotations * 1;
    int sensorCounts = (int)(motorRotations * 2048);
    return sensorCounts;
  }

  public int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(3));
    double motorRotationsPerSecond = wheelRotationsPerSecond * 1;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * 2048);
    return sensorCountsPer100ms;
  }

  public double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / 2048;
    double wheelRotations = motorRotations / 1;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(3));
    return positionMeters;
  }
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = feedForward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = feedForward.calculate(speeds.rightMetersPerSecond);
    double leftOutput =
        m_leftPIDController.calculate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond(), speeds.leftMetersPerSecond);
    double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }
  public void drive(double xSpeed, double rot) {
    setSpeeds(kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      gyro.getRotation2d(), 
      nativeUnitsToDistanceMeters(leftFrontMotor.getSelectedSensorPosition()),
      nativeUnitsToDistanceMeters(rightFrontMotor.getSelectedSensorPosition()));

      field.setRobotPose(odometry.getPoseMeters());
  }
}
