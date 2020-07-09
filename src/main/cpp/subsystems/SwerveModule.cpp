/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule.h"

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include <iostream>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, 
                           int turningMotorChannel,
                           const int driveEncoderPorts[2],
                           const int turningEncoderPorts[2],
                           bool driveEncoderReversed,
                           bool turningEncoderReversed,
                           double offSet,
                           std::string name)
                           
    : m_driveMotor(driveMotorChannel, CANSparkMax::MotorType::kBrushless)
    , m_turningMotor(turningMotorChannel, CANSparkMax::MotorType::kBrushless)
    , m_driveEncoder(m_driveMotor)
    , m_turningEncoder(turningEncoderPorts[0])
    , m_reverseDriveEncoder(driveEncoderReversed)
    , m_reverseTurningEncoder(turningEncoderReversed)
    , m_offSet(offSet)
    , m_name(name)
{
    m_driveEncoder.SetVelocityConversionFactor(wpi::math::pi * ModuleConstants::kWheelDiameterMeters / 60.0); // GetVelocity() will return meters per sec instead of RPM
    //m_driveEncoder.SetVelocityConversionFactor(1.0); // GetVelocity() will return meters per sec instead of RPM

    //m_driveMotor.SetInverted(true);

    m_turningMotor.SetInverted(true);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.EnableContinuousInput(radian_t(-wpi::math::pi), radian_t(wpi::math::pi));
    m_turningPIDController.EnableContinuousInput(-180.0_deg, 180.0_deg);
}

frc::SwerveModuleState SwerveModule::GetState()
{
    double angle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    return {meters_per_second_t{m_driveEncoder.GetVelocity()}, frc::Rotation2d(radian_t(angle))};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState &state)
{
    double p = frc::SmartDashboard::GetNumber("kP", ModuleConstants::kP_ModuleTurningController);
    if (p !=  m_turningPIDController.GetP())
    {
        m_turningPIDController.SetP(p);
    }
    
    double d = frc::SmartDashboard::GetNumber("kd", ModuleConstants::kD_ModuleTurningController);
    if (d !=  m_turningPIDController.GetD())
    {
        m_turningPIDController.SetD(d);
    }

    //m_turningPIDController.SetI(frc::SmartDashboard::GetNumber("kI", 0.000));
    double pDrive = frc::SmartDashboard::GetNumber("kP drive", ModuleConstants::kPModuleDriveController);
    if (p !=  m_drivePIDController.GetP())
    {
        m_drivePIDController.SetP(pDrive);
    }

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput = m_drivePIDController.Calculate(m_driveEncoder.GetVelocity(), state.speed.to<double>());

    // Calculate the turning motor output from the turning PID controller.
    double angle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    auto turnOutput = m_turningPIDController.Calculate(radian_t(angle), state.angle.Radians());

    //std::cout<< " TurnOutput " << turnOutput << " angle " << angle << " state.angle.Radians() " << state.angle.Radians() << "\n";

    std::string dbvalname = m_name + " turnOutput";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), turnOutput);

    dbvalname = m_name + " driveOutput";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), driveOutput);

    dbvalname = m_name + " meas angle";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), angle);
    
    dbvalname = m_name + " cmd angle";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), state.angle.Radians().to<double>());

    dbvalname = m_name + " setpt pos";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), m_turningPIDController.GetSetpoint().position());

    dbvalname = m_name + " setpt velocity";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), m_turningPIDController.GetSetpoint().velocity());

    dbvalname = m_name + " setpt error";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), m_turningPIDController.GetPositionError().to<double>());

    // Set the motor outputs.
    m_driveMotor.Set(driveOutput);
    //m_driveMotor.Set(0);
    m_turningMotor.Set(turnOutput);

    //double out = frc::SmartDashboard::GetNumber("FrontLeft", 0.0);
    //m_turningMotor.Set(out);
}

void SwerveModule::ResetEncoders()
{
    m_driveEncoder.SetPosition(0.0); 
    //m_turningEncoder.SetPosition(0.0);
}

double SwerveModule::VoltageToRadians(double Voltage, double OffSet)
{
    double angle = fmod(Voltage * DriveConstants::kTurnVoltageToRadians - OffSet + 2 * wpi::math::pi, 2 * wpi::math::pi);

    if (angle > wpi::math::pi)
    {
        angle -= 2 * wpi::math::pi;
    }

    return angle;
}