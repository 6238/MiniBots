package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public abstract class BaseDrivetrain extends SubsystemBase {

  public BaseDrivetrain() {
    AutoBuilder.configureLTV(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        Constants.robotPeriod,
        new ReplanningConfig(false, false),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  public abstract void arcadeDrive(double forwardMPS, double rotateRPS);

  public abstract void driveSpeeds(ChassisSpeeds speeds);

  public abstract void driveVoltage(double leftVolts, double rightVolts);

  public abstract void resetEncoders();

  public abstract double getLeftDistanceMeters();

  public abstract double getRightDistanceMeters();

  public abstract Rotation2d getAngle();

  public abstract Pose2d getPose();

  public abstract void resetPose(Pose2d pose);

  public abstract double getLeftSpeed();

  public abstract double getRightSpeed();

  public abstract ChassisSpeeds getChassisSpeeds();

  public abstract double getLeftVolts();

  public abstract double getRightVolts();

  public abstract Field2d getField();

  @Override
  public void periodic() {
    Logger.recordOutput("Drive/Left/Velocity", getLeftSpeed());
    Logger.recordOutput("Drive/Right/Velocity", getRightSpeed());

    Logger.recordOutput("Drive/Left/Distance", getLeftDistanceMeters());
    Logger.recordOutput("Drive/Right/Distance", getRightDistanceMeters());

    Logger.recordOutput("Drive/Left/Voltage", getLeftVolts());
    Logger.recordOutput("Drive/Right/Voltage", getRightVolts());

    Logger.recordOutput("Drive/Angle", getAngle().getDegrees());
    Logger.recordOutput("Drive/Pose", getPose());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
}
