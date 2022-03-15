/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * AP_EFI_Currawong_ECU.cpp
 *
 *      Author: Reilly Callaway
 */
#include "AP_EFI_Currawong_ECU.h"

#if HAL_EFI_CURRAWONG_ECU_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_PiccoloCAN/piccolo_protocol/ECUPackets.h>
#include <AP_Math/definitions.h>

#define KGPM3_TO_GPCM3(x) (0.001f * x)

AP_EFI_Currawong_ECU* AP_EFI_Currawong_ECU::singleton;

AP_EFI_Currawong_ECU::AP_EFI_Currawong_ECU(AP_EFI &_frontend) :
    AP_EFI_Backend(_frontend)
{
    singleton = this;
}

void AP_EFI_Currawong_ECU::update()
{
    // copy the data to the front end
    copy_to_frontend();
}

bool AP_EFI_Currawong_ECU::handle_message(AP_HAL::CANFrame &frame)
{
    bool valid  = true;

    // There are differences between Ardupilot EFI_State and types/scaling of Piccolo packets.
    // So we first decode to Piccolo structs, and then store the data we need in EFI_State internal_state with any scaling required.

    // Structs to decode Piccolo messages into
    ECU_TelemetryFast_t telemetryFast;
    ECU_TelemetrySlow0_t telemetrySlow0;
    ECU_TelemetrySlow1_t telemetrySlow1;
    ECU_TelemetrySlow2_t telemetrySlow2;
    ECU_Errors_t errors;

    // Throw the message at the decoding functions
    if (decodeECU_TelemetryFastPacketStructure(&frame, &telemetryFast))
    {
        internal_state.throttle_position_percent = static_cast<uint8_t>(telemetryFast.throttle);
        // TODO: Do we have a better metric for engine load than just throttle?
        internal_state.engine_load_percent = static_cast<uint8_t>(telemetryFast.throttle);
        internal_state.engine_speed_rpm = static_cast<uint32_t>(telemetryFast.rpm);

        // TODO: Is this appropriate derivation of engine state?
        // Does the ECU provide a starting or fault state?
        if (internal_state.engine_speed_rpm > 0)
        {
            internal_state.engine_state = Engine_State::RUNNING;
        }
        else
        {
            internal_state.engine_state = Engine_State::STOPPED;
        }

        internal_state.estimated_consumed_fuel_volume_cm3 = static_cast<float>(telemetryFast.fuelUsed) / KGPM3_TO_GPCM3(get_ecu_dn());

        internal_state.general_error = telemetryFast.ecuStatusBits.errorIndicator;
        // TODO: Is this needed? Or should we derive engine_state otherwise?
        if (!telemetryFast.ecuStatusBits.enabled)
        {
            internal_state.engine_state = Engine_State::STOPPED;
        }
        
        // Remaining data in packet:
        // ECU_ecuStatusBits_t ecuStatusBits;
    }
    else if (decodeECU_TelemetrySlow0PacketStructure(&frame, &telemetrySlow0))
    {
        internal_state.intake_manifold_pressure_kpa = telemetrySlow0.map;
        internal_state.atmospheric_pressure_kpa = telemetrySlow0.baro;
        internal_state.cylinder_status[0].cylinder_head_temperature = C_TO_KELVIN(telemetrySlow0.cht);

        // Remaining data in packet:
        // uint16_t          rpmCmd;        //!< The reconstructed RPM command
        // ECUThrottleSource throttleSrc;   //!< Source of the throttle information
        // uint16_t          throttlePulse; //!< Throttle pulse width in microseconds
    }
    else if (decodeECU_TelemetrySlow1PacketStructure(&frame, &telemetrySlow1))
    {
        internal_state.intake_manifold_temperature = C_TO_KELVIN(telemetrySlow1.mat);
        internal_state.fuel_pressure = telemetrySlow1.fuelPressure;
        
        // Remaining data in packet:
        // uint32_t        hobbs;        //!< Engine run time in seconds.
        // float           voltage;      //!< Input voltage in Volts
        // ECUGovernorMode governorMode; //!< Operational mode of the governor
    }
    else if (decodeECU_TelemetrySlow2PacketStructure(&frame, &telemetrySlow2))
    {
        internal_state.cylinder_status[0].ignition_timing_deg = telemetrySlow2.ignAngle1;
        if (ENGINE_MAX_CYLINDERS > 1)
        {
            internal_state.cylinder_status[1].ignition_timing_deg = telemetrySlow2.ignAngle2;
        }
        
        internal_state.fuel_consumption_rate_cm3pm = telemetrySlow2.flowRate / KGPM3_TO_GPCM3(get_ecu_dn());

        // TODO: chargeTemp???
        // float cpuLoad;      //!< CPU load in percent
        // float chargeTemp;   //!< Charge temperature in Celsius
        // float injectorDuty; //!< Injector duty cycle in percent
    }
    else if (decodeECU_ErrorsPacketStructure(&frame, &errors))
    {
        // TODO: Do any error bits nicely correspond to any warnings/errors in EFI_State
    }
    else
    {
        valid = false;
    }

    if (valid)
    {
        internal_state.last_updated_ms = AP_HAL::millis();
    }

    return valid;
}

#endif // HAL_EFI_CURRAWONG_ECU_ENABLED