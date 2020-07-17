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

    m_turnPIDController.SetP(ktP);
    m_turnPIDController.SetI(ktI);
    m_turnPIDController.SetD(ktD);
    m_turnPIDController.SetIZone(ktIz);
    m_turnPIDController.SetFF(ktFF);
    m_turnPIDController.SetOutputRange(ktMinOutput, ktMaxOutput);

    m_drivePIDController.SetP(kdP);
    m_drivePIDController.SetD(kdD);
    m_drivePIDController.SetFF(kdFF);
    m_drivePIDController.SetOutputRange(kdMinOutput, kdMaxOutput);

    double initPosition = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    m_turnNeoEncoder.SetPosition(initPosition); // Tell the encoder where the absolute encoder is

    frc::SmartDashboard::PutNumber("Turn P Gain", ktP);
    frc::SmartDashboard::PutNumber("Turn I Gain", ktI);
    frc::SmartDashboard::PutNumber("Turn D Gain", ktD);
    frc::SmartDashboard::PutNumber("Turn I Zone", ktIz);
    frc::SmartDashboard::PutNumber("Turn Feed Forward", ktFF);
    frc::SmartDashboard::PutNumber("Turn Max Output", ktMaxOutput);
    frc::SmartDashboard::PutNumber("Turn Min Output", ktMinOutput);
    frc::SmartDashboard::PutNumber("Set Rotations", 0);

    frc::SmartDashboard::PutNumber("Drive P Gain", kdP);
    frc::SmartDashboard::PutNumber("Drive D Gain", kdD);
    frc::SmartDashboard::PutNumber("Drive Feed Forward", kdFF);
    frc::SmartDashboard::PutNumber("Drive Max Output", kdMaxOutput);
    frc::SmartDashboard::PutNumber("Drive Min Output", kdMinOutput);
}

frc::SwerveModuleState SwerveModule::GetState()
{
    // TODO use the NEO encoder????
    double angle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    return {meters_per_second_t{m_driveEncoder.GetVelocity()}, frc::Rotation2d(radian_t(angle))};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState &state)
{
    // Retrieving turn PID values from SmartDashboard
    double tp = frc::SmartDashboard::GetNumber("Turn P Gain", 0);
    double ti = frc::SmartDashboard::GetNumber("Turn I Gain", 0);
    double td = frc::SmartDashboard::GetNumber("Turn D Gain", 0);
    double tiz = frc::SmartDashboard::GetNumber("Turn I Zone", 0);
    double tff = frc::SmartDashboard::GetNumber("Turn Feed Forward", 0);
    double tmax = frc::SmartDashboard::GetNumber("Turn Max Output", 0);
    double tmin = frc::SmartDashboard::GetNumber("Turn Min Output", 0);
    double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((tp != ktP)) { m_turnPIDController.SetP(tp); ktP = tp; }
    if ((ti != ktI)) { m_turnPIDController.SetI(ti); ktI = ti; }
    if ((td != ktD)) { m_turnPIDController.SetD(td); ktD = td; }
    if ((tiz != ktIz)) { m_turnPIDController.SetIZone(tiz); ktIz = tiz; }
    if ((tff != ktFF)) { m_turnPIDController.SetFF(tff); ktFF = tff; }
    if ((tmax != ktMaxOutput) || (tmin != ktMinOutput))
    { 
        m_turnPIDController.SetOutputRange(tmin, tmax);
        ktMinOutput = tmin; ktMaxOutput = tmax; 
    }

    // Retrieving drive PID values from SmartDashboard
    double dp = frc::SmartDashboard::GetNumber("Drive P Gain", 0);
    double dd = frc::SmartDashboard::GetNumber("Drive D Gain", 0);
    double dff = frc::SmartDashboard::GetNumber("Drive Feed Forward", 0);
    double dmax = frc::SmartDashboard::GetNumber("Drive Max Output", 0);
    double dmin = frc::SmartDashboard::GetNumber("Drive Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((dp != kdP)) { m_turnPIDController.SetP(dp); kdP = dp; }
    if ((dd != kdD)) { m_turnPIDController.SetD(dd); kdD = dd; }
    if ((dff != kdFF)) { m_turnPIDController.SetFF(dff); kdFF = dff; }
    if ((dmax != kdMaxOutput) || (dmin != kdMinOutput))
    { 
        m_drivePIDController.SetOutputRange(dmin, dmax);
        kdMinOutput = dmin; kdMaxOutput = dmax; 
    }

    // Set velocity reference of drivePIDController
    m_drivePIDController.SetReference(state.speed.to<double>(), rev::ControlType::kVelocity);


    // Find absolute encoder and NEO encoder positions
    double absAngle = VoltageToRadians(m_turningEncoder.GetVoltage(), m_offSet);
    double currentPosition = m_turnNeoEncoder.GetPosition();

    // Calculate new turn position given current Neo position, current absolute encoder position, and desired state position
//#define USE_ABS_ENC
#ifdef USE_ABS_ENC
    double minTurnRads = MinTurnRads(absAngle, state.angle.Radians().to<double>());
#else
    double minTurnRads = MinTurnRads(currentPosition, state.angle.Radians().to<double>());
#endif
    double newPosition = currentPosition + minTurnRads;
    // Set position reference of turnPIDController
    m_turnPIDController.SetReference(newPosition, rev::ControlType::kPosition);

    std::string dbvalname;

    dbvalname = m_name + " meas angle";
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), absAngle);

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
    frc::SmartDashboard::PutNumber(dbvalname.c_str(), m_driveMotor.GetAppliedOutput());
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
