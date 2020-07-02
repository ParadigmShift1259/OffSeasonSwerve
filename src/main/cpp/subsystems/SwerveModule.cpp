/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include <iostream>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           const int driveEncoderPorts[2],
                           const int turningEncoderPorts[2],
                           bool driveEncoderReversed,
                           bool turningEncoderReversed,
                           double offSet)
                           
    : m_driveMotor(driveMotorChannel, CANSparkMax::MotorType::kBrushless),
        m_turningMotor(turningMotorChannel, CANSparkMax::MotorType::kBrushless),
        m_driveEncoder(m_driveMotor),
        m_turningEncoder(turningEncoderPorts[0]),
        m_reverseDriveEncoder(driveEncoderReversed),
        m_reverseTurningEncoder(turningEncoderReversed),
        m_offSet(offSet)
{
    m_driveEncoder.SetVelocityConversionFactor(ModuleConstants::kWheelDiameterMeters / 60); // GetVelocity() will return meters per sec instead of RPM

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.EnableContinuousInput(radian_t(-wpi::math::pi), radian_t(wpi::math::pi));
}

frc::SwerveModuleState SwerveModule::GetState()
{
    double angle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    return {meters_per_second_t{m_driveEncoder.GetVelocity()}, frc::Rotation2d(radian_t(angle))};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState &state)
{
    // Calculate the drive output from the drive PID controller.
    const auto driveOutput = m_drivePIDController.Calculate(m_driveEncoder.GetVelocity(), state.speed.to<double>());

    // Calculate the turning motor output from the turning PID controller.
    double angle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    auto turnOutput = m_turningPIDController.Calculate(radian_t(angle), state.angle.Radians());

    std::cout<<"TurnOutput " << turnOutput << "\n";
    // Set the motor outputs.
    //m_driveMotor.Set(driveOutput);
    m_driveMotor.Set(0);
    m_turningMotor.Set(turnOutput);
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