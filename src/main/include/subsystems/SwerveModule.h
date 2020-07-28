/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev\CANSparkMax.h>
#include <rev\CANEncoder.h>

#include <wpi/math>

#include <string>

#include "Constants.h"
#include "Logger.h"

using namespace rev;
using namespace units;

class SwerveModule
{
    using radians_per_second_squared_t = compound_unit<radians, inverse<squared<second>>>;

public:
    SwerveModule(int driveMotorChannel
    , int turningMotorChannel
    , const int driveEncoderPorts[2]
    , const int turningEncoderPorts[2]
    , bool driveEncoderReversed
    , bool turningEncoderReversed
    , double offSet
    , std::string name
    , Logger& log);

    frc::SwerveModuleState GetState();

    void SetDesiredState(frc::SwerveModuleState &state);

    void ResetEncoders();

private:
    double VoltageToRadians(double Voltage, double OffSet);
    double VoltageToDegrees(double Voltage, double OffSet);

    // Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
    double ZeroTo2PiRads(double theta);

    // Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
    double NegPiToPiRads(double theta);

    // Determine the smallest magnitude delta angle that can be added to initial angle that will 
    // result in an angle equivalent (but not necessarily equal) to final angle. 
    // All angles in radians
    double MinTurnRads(double init, double final);

    // We have to use meters here instead of radians due to the fact that
    // ProfiledPIDController's constraints only take in meters per second and
    // meters per second squared.

    static constexpr radians_per_second_t kModuleMaxAngularVelocity = radians_per_second_t(wpi::math::pi);                                           // radians per second
    static constexpr unit_t<radians_per_second_squared_t> kModuleMaxAngularAcceleration = unit_t<radians_per_second_squared_t>(wpi::math::pi * 2.0); // radians per second squared

    CANSparkMax m_driveMotor;
    CANSparkMax m_turningMotor;

    CANPIDController m_drivePIDController = m_driveMotor.GetPIDController();
    CANPIDController m_turnPIDController = m_turningMotor.GetPIDController();

    double kdP = 0.2;
    double kdD = 0.0;
    double kdFF = 0.3;
    double kdMaxOutput = 1.0;
    double kdMinOutput = -1.0;

    double ktP = 0.1;
    double ktI = 0.0;//1e-4;
    double ktD = 1.0;
    double ktIz = 0.0;
    double ktFF = 0.0;
    double ktMaxOutput = 1.0;
    double ktMinOutput = -1.0;

    CANEncoder m_driveEncoder;
    CANEncoder m_turnNeoEncoder = m_turningMotor.GetEncoder();
    frc::AnalogInput m_turningEncoder;

    //frc2::PIDController m_drivePIDController{ModuleConstants::kPModuleDriveController, 0, 0};

    double m_offSet;
    std::string m_name;
    Logger& m_log;
};
