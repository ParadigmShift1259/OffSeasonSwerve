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
    , m_turnNeoEncoder(m_turningMotor, CANEncoder::EncoderType::kHallSensor, 120)
{
    m_driveEncoder.SetVelocityConversionFactor(wpi::math::pi * ModuleConstants::kWheelDiameterMeters / 60.0); // GetVelocity() will return meters per sec instead of RPM
    //m_driveEncoder.SetVelocityConversionFactor(1.0); // GetVelocity() will return meters per sec instead of RPM

    //m_driveMotor.SetInverted(true);

    m_turningMotor.SetInverted(false);

    m_turnPIDController.SetP(kP);
    m_turnPIDController.SetI(kI);
    m_turnPIDController.SetD(kD);
    m_turnPIDController.SetIZone(kIz);
    m_turnPIDController.SetFF(kFF);
    m_turnPIDController.SetOutputRange(kMinOutput, kMaxOutput);

    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("Set Rotations", 0);
}

frc::SwerveModuleState SwerveModule::GetState()
{
    double angle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    return {meters_per_second_t{m_driveEncoder.GetVelocity()}, frc::Rotation2d(radian_t(angle))};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState &state)
{
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_turnPIDController.SetP(p); kP = p; }
    if((i != kI)) { m_turnPIDController.SetI(i); kI = i; }
    if((d != kD)) { m_turnPIDController.SetD(d); kD = d; }
    if((iz != kIz)) { m_turnPIDController.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_turnPIDController.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_turnPIDController.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput = m_drivePIDController.Calculate(m_driveEncoder.GetVelocity(), state.speed.to<double>());

    // Calculate the turning motor output from the turning PID controller.
    double angle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    m_turnPIDController.SetReference(rotations, rev::ControlType::kPosition);
    
    frc::SmartDashboard::PutNumber("SetPoint", rotations);
    frc::SmartDashboard::PutNumber("ProcessVariable", m_turnNeoEncoder.GetPosition());
    //auto turnOutput = m_turningPIDController.Calculate(radian_t(angle), state.angle.Radians());

    //std::cout<< " TurnOutput " << turnOutput << " angle " << angle << " state.angle.Radians() " << state.angle.Radians() << "\n";

    std::string dbvalname = m_name + " driveOutput";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), driveOutput);

    dbvalname = m_name + " meas angle";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), angle);
    
    // Set the motor outputs.
    m_driveMotor.Set(driveOutput);
    //m_driveMotor.Set(0);
    
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