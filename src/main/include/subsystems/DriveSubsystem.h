/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix.h>

#include <ctre/phoenix.h>

#include "Constants.h"
#include "SwerveModule.h"


class DriveSubsystem : public frc2::SubsystemBase
{
public:
    enum EModuleLocation    //!< Order as returned by kDriveKinematics.ToSwerveModuleStates
    {
        eFrontLeft,
        eFrontRight,
        eRearLeft,
        eRearRight
    };
    DriveSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    // Subsystem methods go here.

    /// Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
    /// and the linear speeds have no effect on the angular speed.
    ///
    /// @param xSpeed        Speed of the robot in the x direction
    ///                      (forward/backwards).
    /// @param ySpeed        Speed of the robot in the y direction (sideways).
    /// @param rot           Angular rate of the robot.
    /// @param fieldRelative Whether the provided x and y speeds are relative to the field.
    void Drive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rot, bool fieldRelative);

    /// Resets the drive encoders to currently read a position of 0.
    void ResetEncoders();

    /// Sets the drive SpeedControllers to a power from -1 to 1.
    using SwerveModuleStates = std::array<frc::SwerveModuleState, 4>;
    void SetModuleStates(SwerveModuleStates desiredStates);

    /// Returns the heading of the robot.
    /// @return the robot's heading in degrees, from 180 to 180
    double GetHeading();
    frc::Rotation2d GetHeadingAsRot2d() { return frc::Rotation2d(degree_t(GetHeading())); }

    /// Zeroes the heading of the robot.
    void ZeroHeading();

    /// Returns the turn rate of the robot.
    /// @return The turn rate of the robot, in degrees per second
    double GetTurnRate();

    /// Returns the currently-estimated pose of the robot.
    /// @return The pose.
    frc::Pose2d GetPose();

    /// Resets the odometry to the specified pose.
    /// @param pose The pose to which to set the odometry.
    void ResetOdometry(frc::Pose2d pose);

    meter_t kTrackWidth = 21.5_in; // Distance between centers of right and left wheels on robot
    meter_t kWheelBase = 23.5_in;  // Distance between centers of front and back wheels on robot

    frc::SwerveDriveKinematics<4> kDriveKinematics{
        frc::Translation2d(kWheelBase / 2, kTrackWidth / 2),    // +x, +y FL
        frc::Translation2d(kWheelBase / 2, -kTrackWidth / 2),   // +x, -y FR
        frc::Translation2d(-kWheelBase / 2, kTrackWidth / 2),   // -x, +y RL
        frc::Translation2d(-kWheelBase / 2, -kTrackWidth / 2)}; // -x, -y RR

private:
    SwerveModule m_frontLeft;
    SwerveModule m_frontRight;
    SwerveModule m_rearLeft;
    SwerveModule m_rearRight;

    // The gyro sensor
    //frc::ADXRS450_Gyro m_gyro;

    // Odometry class for tracking robot pose
    // 4 defines the number of modules
    frc::SwerveDriveOdometry<4> m_odometry;
    //ctre::PigeonIMU m_gyro;
};
