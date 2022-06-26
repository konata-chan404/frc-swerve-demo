// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.hal.simulation.SpiReadAutoReceiveBufferCallback;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class SwerveModule {
    private static final double wheelRadius = 0.0508;
    private static final int encoderResolution = 2048;

    private final static double distancePerPulse = (2 * Math.PI * wheelRadius) / encoderResolution;
    private final static double anglePerPulse = (2 * Math.PI) / encoderResolution;

    private static final double moduleMaxAngularVelocity = DrivetrainSubsystem.maxAngularVelocity;
    private static final double modduleMaxAngularAcceleration = 2 * Math.PI;

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController drivePID = new PIDController(1, 0, 0);
    private final ProfiledPIDController turningPID = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(moduleMaxAngularVelocity, modduleMaxAngularAcceleration));

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(1, 0.5);

    public SwerveModule(int driveID, int turningID) {
        driveMotor = new TalonFX(driveID);
        turningMotor = new TalonFX(turningID);

        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getSpeed() {
        return driveMotor.getSelectedSensorVelocity() * distancePerPulse;
    }

    public double getAngle() {
        return turningMotor.getSelectedSensorPosition() * anglePerPulse;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), new Rotation2d(getAngle()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()));

        double driveOutput = drivePID.calculate(getSpeed(), optimizedState.speedMetersPerSecond);
        double driveFeed = driveFeedforward.calculate(optimizedState.speedMetersPerSecond);

        double turnOutput = turningPID.calculate(getAngle(), optimizedState.angle.getRadians());
        double turnFeed = turningFeedforward.calculate(turningPID.getSetpoint().velocity);

        driveMotor.set(ControlMode.Current, driveOutput + driveFeed);
        turningMotor.set(ControlMode.Current, turnOutput + turnFeed);
    }
}
