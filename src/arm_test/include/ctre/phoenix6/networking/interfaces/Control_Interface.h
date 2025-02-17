/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
#pragma once

#include "ctre/phoenix/export.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
    CTREXPORT int c_ctre_phoenix6_requestConfigApply(const char *canbus, uint32_t ecuEncoding, double timeoutSeconds, const char *str, uint32_t strlen, bool forceApply);
    CTREXPORT int c_ctre_phoenix6_RequestControlEmpty(const char *canbus, uint32_t ecuEncoding, double updateTime);
CTREXPORT int c_ctre_phoenix6_RequestControlDutyCycleOut(const char *canbus, uint32_t ecuEncoding, double updateTime, double Output, bool EnableFOC, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlTorqueCurrentFOC(const char *canbus, uint32_t ecuEncoding, double updateTime, double Output, double MaxAbsDutyCycle, double Deadband, bool OverrideCoastDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlVoltageOut(const char *canbus, uint32_t ecuEncoding, double updateTime, double Output, bool EnableFOC, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlPositionDutyCycle(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, double Velocity, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlPositionVoltage(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, double Velocity, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlPositionTorqueCurrentFOC(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, double Velocity, double FeedForward, int Slot, bool OverrideCoastDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlVelocityDutyCycle(const char *canbus, uint32_t ecuEncoding, double updateTime, double Velocity, double Acceleration, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlVelocityVoltage(const char *canbus, uint32_t ecuEncoding, double updateTime, double Velocity, double Acceleration, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlVelocityTorqueCurrentFOC(const char *canbus, uint32_t ecuEncoding, double updateTime, double Velocity, double Acceleration, double FeedForward, int Slot, bool OverrideCoastDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlMotionMagicDutyCycle(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlMotionMagicVoltage(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlMotionMagicTorqueCurrentFOC(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, double FeedForward, int Slot, bool OverrideCoastDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDifferentialDutyCycle(const char *canbus, uint32_t ecuEncoding, double updateTime, double TargetOutput, double DifferentialPosition, bool EnableFOC, int DifferentialSlot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDifferentialVoltage(const char *canbus, uint32_t ecuEncoding, double updateTime, double TargetOutput, double DifferentialPosition, bool EnableFOC, int DifferentialSlot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDifferentialPositionDutyCycle(const char *canbus, uint32_t ecuEncoding, double updateTime, double TargetPosition, double DifferentialPosition, bool EnableFOC, int TargetSlot, int DifferentialSlot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDifferentialPositionVoltage(const char *canbus, uint32_t ecuEncoding, double updateTime, double TargetPosition, double DifferentialPosition, bool EnableFOC, int TargetSlot, int DifferentialSlot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDifferentialVelocityDutyCycle(const char *canbus, uint32_t ecuEncoding, double updateTime, double TargetVelocity, double DifferentialPosition, bool EnableFOC, int TargetSlot, int DifferentialSlot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDifferentialVelocityVoltage(const char *canbus, uint32_t ecuEncoding, double updateTime, double TargetVelocity, double DifferentialPosition, bool EnableFOC, int TargetSlot, int DifferentialSlot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDifferentialMotionMagicDutyCycle(const char *canbus, uint32_t ecuEncoding, double updateTime, double TargetPosition, double DifferentialPosition, bool EnableFOC, int TargetSlot, int DifferentialSlot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDifferentialMotionMagicVoltage(const char *canbus, uint32_t ecuEncoding, double updateTime, double TargetPosition, double DifferentialPosition, bool EnableFOC, int TargetSlot, int DifferentialSlot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlFollower(const char *canbus, uint32_t ecuEncoding, double updateTime, int MasterID, bool OpposeMasterDirection);
CTREXPORT int c_ctre_phoenix6_RequestControlStrictFollower(const char *canbus, uint32_t ecuEncoding, double updateTime, int MasterID);
CTREXPORT int c_ctre_phoenix6_RequestControlDifferentialFollower(const char *canbus, uint32_t ecuEncoding, double updateTime, int MasterID, bool OpposeMasterDirection);
CTREXPORT int c_ctre_phoenix6_RequestControlDifferentialStrictFollower(const char *canbus, uint32_t ecuEncoding, double updateTime, int MasterID);
CTREXPORT int c_ctre_phoenix6_RequestControlNeutralOut(const char *canbus, uint32_t ecuEncoding, double updateTime, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlCoastOut(const char *canbus, uint32_t ecuEncoding, double updateTime, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlStaticBrake(const char *canbus, uint32_t ecuEncoding, double updateTime, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlMusicTone(const char *canbus, uint32_t ecuEncoding, double updateTime, double AudioFrequency);
CTREXPORT int c_ctre_phoenix6_RequestControlMotionMagicVelocityDutyCycle(const char *canbus, uint32_t ecuEncoding, double updateTime, double Velocity, double Acceleration, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlMotionMagicVelocityTorqueCurrentFOC(const char *canbus, uint32_t ecuEncoding, double updateTime, double Velocity, double Acceleration, bool EnableFOC, double FeedForward, int Slot, bool OverrideCoastDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlMotionMagicVelocityVoltage(const char *canbus, uint32_t ecuEncoding, double updateTime, double Velocity, double Acceleration, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlMotionMagicExpoDutyCycle(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlMotionMagicExpoVoltage(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlMotionMagicExpoTorqueCurrentFOC(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, double FeedForward, int Slot, bool OverrideCoastDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDynamicMotionMagicDutyCycle(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, double Velocity, double Acceleration, double Jerk, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDynamicMotionMagicVoltage(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, double Velocity, double Acceleration, double Jerk, bool EnableFOC, double FeedForward, int Slot, bool OverrideBrakeDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDynamicMotionMagicTorqueCurrentFOC(const char *canbus, uint32_t ecuEncoding, double updateTime, double Position, double Velocity, double Acceleration, double Jerk, double FeedForward, int Slot, bool OverrideCoastDurNeutral, bool LimitForwardMotion, bool LimitReverseMotion, bool IgnoreHardwareLimits, bool UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_DutyCycleOut_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Output, bool AverageRequest_EnableFOC, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_PositionDutyCycle_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, double AverageRequest_Velocity, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_VelocityDutyCycle_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Velocity, double AverageRequest_Acceleration, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_MotionMagicDutyCycle_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_DutyCycleOut_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Output, bool AverageRequest_EnableFOC, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_PositionDutyCycle_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, double AverageRequest_Velocity, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_VelocityDutyCycle_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Velocity, double AverageRequest_Acceleration, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_MotionMagicDutyCycle_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_VoltageOut_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Output, bool AverageRequest_EnableFOC, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_PositionVoltage_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, double AverageRequest_Velocity, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_VelocityVoltage_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Velocity, double AverageRequest_Acceleration, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_MotionMagicVoltage_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_VoltageOut_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Output, bool AverageRequest_EnableFOC, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_PositionVoltage_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, double AverageRequest_Velocity, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_VelocityVoltage_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Velocity, double AverageRequest_Acceleration, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_MotionMagicVoltage_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, bool AverageRequest_EnableFOC, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideBrakeDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, bool DifferentialRequest_EnableFOC, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideBrakeDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_TorqueCurrentFOC_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Output, double AverageRequest_MaxAbsDutyCycle, double AverageRequest_Deadband, bool AverageRequest_OverrideCoastDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideCoastDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_PositionTorqueCurrentFOC_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, double AverageRequest_Velocity, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideCoastDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideCoastDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_VelocityTorqueCurrentFOC_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Velocity, double AverageRequest_Acceleration, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideCoastDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideCoastDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_MotionMagicTorqueCurrentFOC_Position(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideCoastDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Position, double DifferentialRequest_Velocity, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideCoastDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_TorqueCurrentFOC_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Output, double AverageRequest_MaxAbsDutyCycle, double AverageRequest_Deadband, bool AverageRequest_OverrideCoastDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideCoastDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_PositionTorqueCurrentFOC_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, double AverageRequest_Velocity, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideCoastDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideCoastDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_VelocityTorqueCurrentFOC_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Velocity, double AverageRequest_Acceleration, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideCoastDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideCoastDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
CTREXPORT int c_ctre_phoenix6_RequestControlDiff_MotionMagicTorqueCurrentFOC_Velocity(const char *canbus, uint32_t ecuEncoding, double updateTime, double AverageRequest_Position, double AverageRequest_FeedForward, int AverageRequest_Slot, bool AverageRequest_OverrideCoastDurNeutral, bool AverageRequest_LimitForwardMotion, bool AverageRequest_LimitReverseMotion, bool AverageRequest_IgnoreHardwareLimits, bool AverageRequest_UseTimesync, double DifferentialRequest_Velocity, double DifferentialRequest_Acceleration, double DifferentialRequest_FeedForward, int DifferentialRequest_Slot, bool DifferentialRequest_OverrideCoastDurNeutral, bool DifferentialRequest_LimitForwardMotion, bool DifferentialRequest_LimitReverseMotion, bool DifferentialRequest_IgnoreHardwareLimits, bool DifferentialRequest_UseTimesync);
#ifdef __cplusplus
}
#endif
