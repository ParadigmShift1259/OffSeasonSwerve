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
    , m_turnNeoEncoder(m_turningMotor)
{
    m_driveMotor.SetSmartCurrentLimit(ModuleConstants::kMotorCurrentLimit);
    m_turningMotor.SetSmartCurrentLimit(ModuleConstants::kMotorCurrentLimit);

    m_driveMotor.SetOpenLoopRampRate(0.1);
    m_turningMotor.SetOpenLoopRampRate(0.1);

    m_driveEncoder.SetVelocityConversionFactor(wpi::math::pi * ModuleConstants::kWheelDiameterMeters / 60.0); // GetVelocity() will return meters per sec instead of RPM
    m_turnNeoEncoder.SetPositionConversionFactor(2 * wpi::math::pi / 18.0); // 18 motor revolutions per wheel revolution
    
    //m_driveEncoder.SetVelocityConversionFactor(1.0); // GetVelocity() will return meters per sec instead of RPM
    //m_driveMotor.SetInverted(true);
    m_turningMotor.SetInverted(false);

    m_turnPIDController.SetP(kP);
    m_turnPIDController.SetI(kI);
    m_turnPIDController.SetD(kD);
    m_turnPIDController.SetIZone(kIz);
    m_turnPIDController.SetFF(kFF);
    m_turnPIDController.SetOutputRange(kMinOutput, kMaxOutput);

    double initPosition = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    m_turnNeoEncoder.SetPosition(initPosition); // Tell the encoder where the absolute encoder is

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
    // TODO use the NEO encoder????
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
    if ((p != kP)) { m_turnPIDController.SetP(p); kP = p; }
    if ((i != kI)) { m_turnPIDController.SetI(i); kI = i; }
    if ((d != kD)) { m_turnPIDController.SetD(d); kD = d; }
    if ((iz != kIz)) { m_turnPIDController.SetIZone(iz); kIz = iz; }
    if ((ff != kFF)) { m_turnPIDController.SetFF(ff); kFF = ff; }
    if ((max != kMaxOutput) || (min != kMinOutput))
    { 
        m_turnPIDController.SetOutputRange(min, max);
        kMinOutput = min; kMaxOutput = max; 
    }

    std::string dbvalname;

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput = m_drivePIDController.Calculate(m_driveEncoder.GetVelocity(), state.speed.to<double>());

    double currentPosition = m_turnNeoEncoder.GetPosition();

    // Calculate the turning motor output from the turning PID controller.

    double absAngle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    dbvalname = m_name + " meas angle";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), absAngle);
//#define USE_ABS_ENC
#ifdef USE_ABS_ENC
    double minTurnRads = MinTurnRads(absAngle, state.angle.Radians().to<double>());
#else
    double minTurnRads = MinTurnRads(currentPosition, state.angle.Radians().to<double>());
#endif
    double newPosition = currentPosition + minTurnRads;
    m_turnPIDController.SetReference(newPosition, rev::ControlType::kPosition);

    dbvalname = m_name + " minTurnRads";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), minTurnRads);
    dbvalname = m_name + " currentPosition";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), currentPosition);
    dbvalname = m_name + " newPosition";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), newPosition);

    dbvalname = m_name + " SetPoint";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), newPosition);
    dbvalname = m_name + " ProcessVariable";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), m_turnNeoEncoder.GetPosition());

    dbvalname = m_name + " driveOutput";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), driveOutput);

    // Set the motor outputs.
    m_driveMotor.Set(driveOutput);
}

void SwerveModule::ResetEncoders()
{
    m_driveEncoder.SetPosition(0.0); 
}

double SwerveModule::VoltageToRadians(double Voltage, double OffSet)
{
    double angle = fmod(Voltage * DriveConstants::kTurnVoltageToRadians - OffSet + 2 * wpi::math::pi, 2 * wpi::math::pi);
    angle = 2 * wpi::math::pi - angle;

    // if (angle > wpi::math::pi)
    // {
    //     angle -= 2 * wpi::math::pi;
    // }

    return angle;
}

double SwerveModule::VoltageToDegrees(double voltage, double offSet)
{
    double angle = fmod(voltage * DriveConstants::KTurnVoltageToDegrees - offSet + 360.0, 360.0);

    //if (angle > 180.0)
    // {
    //     angle -= 360.0;
    // }

    return angle;
}

// Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
double SwerveModule::ZeroTo2PiRads(double theta)
    {
    theta = fmod(theta, 2 * M_PI);
    if (theta < 0)
        theta += 2 * M_PI;
        
    return theta;
    }

// Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
double SwerveModule::NegPiToPiRads(double theta)
{
    theta = ZeroTo2PiRads(theta);
    if (theta > M_PI)
        theta -= 2 * M_PI;
        
    return theta;
}

// Determine the smallest magnitude delta angle that can be added to initial angle that will 
// result in an angle equivalent (but not necessarily equal) to final angle. 
// All angles in radians
double SwerveModule::MinTurnRads(double init, double final)
{
    init = ZeroTo2PiRads(init);
    final = ZeroTo2PiRads(final);

    double turn = final - init;
    if (turn > M_PI)
        turn -= 2 * M_PI;
    else if (turn < -M_PI)
        turn += 2 * M_PI;
        
    return turn;
}
