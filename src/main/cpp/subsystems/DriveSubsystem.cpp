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
#include <iostream>
#include <frc/SmartDashBoard/SmartDashboard.h>

using namespace DriveConstants;
using namespace std;
using namespace frc;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{
        kFrontLeftDriveMotorPort,
        kFrontLeftTurningMotorPort,
        kFrontLeftDriveEncoderPorts,
        kFrontLeftTurningEncoderPorts,
        kFrontLeftDriveMotorReversed,
        kFrontLeftTurningEncoderReversed,
        kFrontLeftOffset,
        std::string("FrontLeft")
        },

      m_frontRight{
        kFrontRightDriveMotorPort,
        kFrontRightTurningMotorPort,
        kFrontRightDriveEncoderPorts,
        kFrontRightTurningEncoderPorts,
        kFrontRightDriveMotorReversed,
        kFrontRightTurningEncoderReversed,
        kFrontRightOffset,
        std::string("FrontRight")
        },

      m_rearRight{
        kRearRightDriveMotorPort,
        kRearRightTurningMotorPort,
        kRearRightDriveEncoderPorts,
        kRearRightTurningEncoderPorts,
        kRearRightDriveMotorReversed,
        kRearRightTurningEncoderReversed,
        kRearRightOffset,
        std::string("RearRight")
        },

      m_rearLeft{
        kRearLeftDriveMotorPort,
        kRearLeftTurningMotorPort,
        kRearLeftDriveEncoderPorts,
        kRearLeftTurningEncoderPorts,
        kRearLeftDriveMotorReversed,
        kRearLeftTurningEncoderReversed,
        kRearLeftOffset,
        std::string("RearLeft")
        },

      m_odometry{kDriveKinematics, GetHeadingAsRot2d(), frc::Pose2d()}
      //,m_gyro(0)
{
    SmartDashboard::PutBoolean("GetInputFromNetTable", true);

    SmartDashboard::PutNumber("FrontLeft", 0.0);
    SmartDashboard::PutNumber("FrontRight", 0.0);
    SmartDashboard::PutNumber("RearRight", 0.0);
    SmartDashboard::PutNumber("RearLeft", 0.0);
    
    SmartDashboard::PutNumber("FrontLeftV", 0.0);
    SmartDashboard::PutNumber("FrontRightV", 0.0);
    SmartDashboard::PutNumber("RearLeftV", 0.0);
    SmartDashboard::PutNumber("RearRightV", 0.0);

    SmartDashboard::PutNumber("kP", ModuleConstants::kP_ModuleTurningController);
    SmartDashboard::PutNumber("kd", ModuleConstants::kD_ModuleTurningController);
    SmartDashboard::PutNumber("kI", 0.000);

    SmartDashboard::PutNumber("kP drive", ModuleConstants::kPModuleDriveController);

    SmartDashboard::PutNumber("Tolerance", 0.1);
}

void DriveSubsystem::Periodic()
{
    // Implementation of subsystem periodic method goes here.
    m_odometry.Update(GetHeadingAsRot2d()
                    , m_frontLeft.GetState()
                    , m_rearLeft.GetState() // TODO check order FL, RL?
                    , m_frontRight.GetState()
                    , m_rearRight.GetState());
}

void DriveSubsystem::Drive(meters_per_second_t xSpeed, meters_per_second_t ySpeed, radians_per_second_t rot, bool fieldRelative)
{
    SmartDashboard::PutNumber("x speed", xSpeed.to<double>());
    SmartDashboard::PutNumber("y speed", ySpeed.to<double>());
    SmartDashboard::PutNumber("rotation", rot.to<double>());

    frc::ChassisSpeeds chassisSpeeds;
    if (fieldRelative)
        chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetHeadingAsRot2d());
    else
        chassisSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, rot};

    auto states = kDriveKinematics.ToSwerveModuleStates(chassisSpeeds);

    kDriveKinematics.NormalizeWheelSpeeds(&states, AutoConstants::kMaxSpeed);
    
    if (SmartDashboard::GetBoolean("GetInputFromNetTable", true))
    {
        double angle = SmartDashboard::GetNumber("FrontLeft", 0.0);
        states[eFrontLeft].angle = frc::Rotation2d(radian_t(angle));

        angle = SmartDashboard::GetNumber("FrontRight", 0.0);
        states[eFrontRight].angle = frc::Rotation2d(radian_t(angle));

        angle = SmartDashboard::GetNumber("RearLeft", 0.0);
        states[eRearLeft].angle = frc::Rotation2d(radian_t(angle));

        angle = SmartDashboard::GetNumber("RearRight", 0.0);
        states[eRearRight].angle = frc::Rotation2d(radian_t(angle));

        double speed = SmartDashboard::GetNumber("FrontLeftV", 0.0);
        states[eFrontLeft].speed = meters_per_second_t(speed);

        speed = SmartDashboard::GetNumber("FrontRightV", 0.0);
        states[eFrontLeft].speed = meters_per_second_t(speed);

        speed = SmartDashboard::GetNumber("RearLeftV", 0.0);
        states[eFrontLeft].speed = meters_per_second_t(speed);

        speed = SmartDashboard::GetNumber("RearRightV", 0.0);
        states[eFrontLeft].speed = meters_per_second_t(speed);
    }

    // cout<<"Drive() ";
    // cout<<" FrontLeft Angle: "<< states[eFrontLeft].angle.Radians() <<" FrontLeft Speed: "<< (double)states[eFrontLeft].speed;
    // cout<<" FrontRight Angle: "<< states[eFrontRight].angle.Radians() <<" FrontRight Speed: "<< (double)states[eFrontRight].speed;
    // cout<<" RearRight Angle: "<< states[eRearRight].angle.Radians() <<" RearRight Speed: "<< (double)states[eRearRight].speed;
    // cout<<" RearLeft Angle: "<< states[eRearLeft].angle.Radians() <<" RearLeft Speed: "<< (double)states[eRearLeft].speed;
    // cout<<"\n";
    m_frontLeft.SetDesiredState(states[eFrontLeft]);
    m_frontRight.SetDesiredState(states[eFrontRight]);
    m_rearLeft.SetDesiredState(states[eRearLeft]);
    m_rearRight.SetDesiredState(states[eRearRight]);
}

void DriveSubsystem::SetModuleStates(SwerveModuleStates desiredStates)
{
    kDriveKinematics.NormalizeWheelSpeeds(&desiredStates, AutoConstants::kMaxSpeed);
    cout<<"SetModuleStates() ";
    cout<<" FrontLeft Angle: "<< desiredStates[eFrontLeft].angle.Radians() <<" FrontLeft Speed: "<< (double)desiredStates[eFrontLeft].speed;
    cout<<" FrontRight Angle: "<< desiredStates[eFrontRight].angle.Radians() <<" FrontRight Speed: "<< (double)desiredStates[eFrontRight].speed;
    cout<<" RearRight Angle: "<< desiredStates[eRearRight].angle.Radians() <<" RearRight Speed: "<< (double)desiredStates[eRearRight].speed;
    cout<<" RearLeft Angle: "<< desiredStates[eRearLeft].angle.Radians() <<" RearLeft Speed: "<< (double)desiredStates[eRearLeft].speed;
    cout<<"\n";
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
    return 0.0; //std::remainder(m_gyro.GetAngle(), 360) * (kGyroReversed ? -1. : 1.);
}

void DriveSubsystem::ZeroHeading()
{
    //m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate()
{
    return 0.0; //m_gyro.GetRate() * (kGyroReversed ? -1. : 1.);
}

frc::Pose2d DriveSubsystem::GetPose()
{
    return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
    m_odometry.ResetPosition(pose, GetHeadingAsRot2d());
}
