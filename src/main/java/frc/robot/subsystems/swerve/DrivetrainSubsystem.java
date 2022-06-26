// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  public final static double maxVelocity = 3;
  public final static double maxAngularVelocity = Math.PI;

  private final Translation2d frontLeftLocation = new Translation2d(0.35, 0.35);
  private final Translation2d frontRightLocation = new Translation2d(0.35, -0.35);
  private final Translation2d rearLeftLocation = new Translation2d(-0.35, 0.35);
  private final Translation2d rearRightLocation = new Translation2d(-0.35, -0.35);

  private final SwerveModule frontLeftModule = new SwerveModule(0, 1);
  private final SwerveModule frontRightModule = new SwerveModule(2, 3);
  private final SwerveModule rearLeftModule = new SwerveModule(4, 5);
  private final SwerveModule rearRightModule = new SwerveModule(6, 7);

  private final AHRS navx = new AHRS(Port.kOnboard);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
      rearLeftLocation, rearRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(navx.getAngle()));

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    navx.reset();
  }

  public void drive(double x, double y, double rotation, boolean fieldRelative) {
    SwerveModuleState[] states = kinematics
        .toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, new Rotation2d(navx.getAngle()))
                : new ChassisSpeeds(x, y, rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);

    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    rearLeftModule.setDesiredState(states[3]);
    rearRightModule.setDesiredState(states[4]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(new Rotation2d(navx.getAngle()), frontLeftModule.getState(),
        frontRightModule.getState(), rearLeftModule.getState(), rearRightModule.getState());
  }
}
