// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class XRPDrivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // private final MutableMeasure<Voltage> m_appliedVoltage = Volts.of(0).mutableCopy();
  // private final MutableMeasure<Distance> m_distance = Inches.of(0).mutableCopy();
  // private final MutableMeasure<Velocity<Distance>> m_velocity = InchesPerSecond.of(0).mutableCopy();

  private double left_ks = 2.5526;
  private double left_kv = 0.3147;
  private double left_ka = 0;

  private double right_ks = 4.0322;
  private double right_kv = 0;
  private double right_ka = 0.41325;

  private SimpleMotorFeedforward leftMotorFF = new SimpleMotorFeedforward(left_ks, left_kv, left_ka);
  private SimpleMotorFeedforward rightMotorFF = new SimpleMotorFeedforward(right_ks, right_kv, right_ka);

  // private ArrayList<Double> leftPositionBuffer = new ArrayList<>();
  // private ArrayList<Double> rightPositionBuffer = new ArrayList<>();

  // private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
  //   new SysIdRoutine.Config(),
  //   new SysIdRoutine.Mechanism(
  //     voltage -> {
  //       m_leftMotor.setVoltage(voltage.magnitude());
  //       m_rightMotor.setVoltage(voltage.magnitude());
  //     },
  //     log -> {
  //       leftPositionBuffer.add(m_leftEncoder.getDistance());
  //       rightPositionBuffer.add(m_rightEncoder.getDistance());
  //       if (leftPositionBuffer.size() > 2) {
  //         leftPositionBuffer.remove(0);
  //       }
  //       if (rightPositionBuffer.size() > 2) {
  //         rightPositionBuffer.remove(0);
  //       }
  //       double leftSpeed = (leftPositionBuffer.get(leftPositionBuffer.size()-1) - leftPositionBuffer.get(0)) / (0.02 * leftPositionBuffer.size());
  //       double rightSpeed = (rightPositionBuffer.get(rightPositionBuffer.size()-1) - rightPositionBuffer.get(0)) / (0.02 * rightPositionBuffer.size());

  //       log.motor("drive-left")
  //           .voltage(
  //               m_appliedVoltage.mut_replace(
  //                   m_leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
  //           .linearPosition(m_distance.mut_replace(m_leftEncoder.getDistance(), Inches))
  //           .linearVelocity(
  //               m_velocity.mut_replace(leftSpeed, InchesPerSecond));
  //       log.motor("drive-right")
  //           .voltage(
  //               m_appliedVoltage.mut_replace(
  //                   m_rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
  //           .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Inches))
  //           .linearVelocity(m_velocity.mut_replace(rightSpeed, InchesPerSecond));
  //     },
  //     this
  //   )
  // );

  /** Creates a new XRPDrivetrain. */
  public XRPDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch * 40) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch * 40) / kCountsPerRevolution);
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void forward(double speed) {
    m_leftMotor.setVoltage(leftMotorFF.calculate(speed));
    m_rightMotor.setVoltage(rightMotorFF.calculate(speed));
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return m_SysIdRoutine.quasistatic(direction);
  // }
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return m_SysIdRoutine.dynamic(direction);
  // }
}
