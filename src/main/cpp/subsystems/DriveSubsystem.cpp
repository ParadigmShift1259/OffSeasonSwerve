/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/units.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{ kFrontLeftDriveMotorPort
                  ,kFrontLeftTurningMotorPort
                  ,kFrontLeftDriveEncoderPorts
                  ,kFrontLeftTurningEncoderPorts
                  ,kFrontLeftDriveEncoderReversed
                  ,kFrontLeftTurningEncoderReversed},

      m_frontRight{ kFrontRightDriveMotorPort
                  , kFrontRightTurningMotorPort
                  , kFrontRightDriveEncoderPorts
                  , kFrontRightTurningEncoderPorts
                  , kFrontRightDriveEncoderReversed
                  , kFrontRightTurningEncoderReversed},

      m_rearRight{  kRearRightDriveMotorPort
                  , kRearRightTurningMotorPort
                  , kRearRightDriveEncoderPorts
                  , kRearRightTurningEncoderPorts
                  , kRearRightDriveEncoderReversed
                  , kRearRightTurningEncoderReversed},

      m_rearLeft{ kRearLeftDriveMotorPort
                , kRearLeftTurningMotorPort
                , kRearLeftDriveEncoderPorts
                , kRearLeftTurningEncoderPorts
                , kRearLeftDriveEncoderReversed
                , kRearLeftTurningEncoderReversed},

      m_odometry{kDriveKinematics, GetHeadingAsRot2d(), frc::Pose2d()}
{
}

void DriveSubsystem::Periodic()
{
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(  GetHeadingAsRot2d()
                    , m_frontLeft.GetState()
                    , m_rearLeft.GetState()   // TODO check order FL, RL?
                    , m_frontRight.GetState()
                    , m_rearRight.GetState());
}

void DriveSubsystem::Drive(  meters_per_second_t xSpeed
                           , meters_per_second_t ySpeed
                           , radians_per_second_t rot
                           , bool fieldRelative)
{
  frc::ChassisSpeeds chassisSpeeds;
  if (fieldRelative)
  {
    chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetHeadingAsRot2d());
  }
  else
  {
    chassisSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, rot};
  }
  auto states = kDriveKinematics.ToSwerveModuleStates(chassisSpeeds);

  kDriveKinematics.NormalizeWheelSpeeds(&states, AutoConstants::kMaxSpeed);
      
  m_frontLeft.SetDesiredState(states[eFrontLeft]);
  m_frontRight.SetDesiredState(states[eFrontRight]);
  m_rearLeft.SetDesiredState(states[eRearRight]);
  m_rearRight.SetDesiredState(states[eRearLeft]);
}

void DriveSubsystem::SetModuleStates(SwerveModuleStates desiredStates) 
{
  kDriveKinematics.NormalizeWheelSpeeds(&desiredStates, AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[eFrontLeft]);
  m_frontRight.SetDesiredState(desiredStates[eFrontRight]);
  m_rearRight.SetDesiredState(desiredStates[eRearRight]);
  m_rearLeft.SetDesiredState(desiredStates[eRearLeft]);
}

void DriveSubsystem::ResetEncoders() 
{
  m_frontLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
  m_rearLeft.ResetEncoders();
}

double DriveSubsystem::GetHeading() 
{
  return 0.0;//std::remainder(m_gyro.GetAngle(), 360) * (kGyroReversed ? -1. : 1.);
}

void DriveSubsystem::ZeroHeading()
{
   //m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate()
{
  return 0.0;//m_gyro.GetRate() * (kGyroReversed ? -1. : 1.);
}

frc::Pose2d DriveSubsystem::GetPose()
{
   return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
  m_odometry.ResetPosition(pose, GetHeadingAsRot2d());
}
