/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#include "AP_PiccoloCAN_ESC.h"

#if HAL_PICCOLO_CAN_ENABLE

/*
 * Decode a received CAN frame.
 * It is assumed at this point that the received frame is intended for *this* ESC
 */
bool AP_PiccoloCAN_ESC::handle_can_frame(AP_HAL::CANFrame &frame)
{
    bool result = true;

    // The ESC address is the lower byte of the frame ID
    uint8_t addr = frame.id & 0xFF;

    // Ignore any ESC with node ID of zero
    if (addr == 0x00) {
        return false;
    }

    addr -= 1;

    uint8_t extended;

    if (decodeESC_StatusAPacketStructure(&frame, &status.statusA)) {
#if AP_EXTENDED_ESC_TELEM_ENABLED
        AP_ESC_Telem_Backend::TelemetryData telem {};

        if ((status.statusA.mode & 0xf) == ESC_MODE_PWM)
        {
            telem.input_duty = uint8_t(inputPercent() + 0.5f);      // Round instead of truncate
            update_telem_data(addr, telem, AP_ESC_Telem_Backend::TelemetryType::INPUT_DUTY);
        }
#endif // AP_EXTENDED_ESC_TELEM_ENABLED
        update_rpm(addr, rpm());
        newTelemetry = true;
    } else if (decodeESC_StatusBPacketStructure(&frame, &status.statusB)) {
        AP_ESC_Telem_Backend::TelemetryData telem {};

        telem.voltage = voltage() * 10;
        telem.current = current() * 10;
        telem.motor_temp_cdeg = int16_t(motorTemperature() * 100);
        telem.temperature_cdeg = int16_t(temperature() * 100);
#if AP_EXTENDED_ESC_TELEM_ENABLED
        telem.output_duty = uint8_t(dutyCycle() + 0.5f);            // Round instead of truncate
#endif // AP_EXTENDED_ESC_TELEM_ENABLED

        update_telem_data(addr, telem,
            AP_ESC_Telem_Backend::TelemetryType::CURRENT
            | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
            | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE
            | AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE
#if AP_EXTENDED_ESC_TELEM_ENABLED
            | AP_ESC_Telem_Backend::TelemetryType::OUTPUT_DUTY
#endif // AP_EXTENDED_ESC_TELEM_ENABLED
            );

        newTelemetry = true;
    } else if (decodeESC_StatusCPacketStructure(&frame, &status.statusC)) {
        AP_ESC_Telem_Backend::TelemetryData telem {};

        telem.temperature_cdeg = temperature() * 100;
        update_telem_data(addr, telem, AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
        newTelemetry = true;
    } else if (decodeESC_WarningErrorStatusPacket(&frame, &status.warnings, &status.errors, &extended, &status.warnings, &status.errors)) {
#if AP_EXTENDED_ESC_TELEM_ENABLED
        AP_ESC_Telem_Backend::TelemetryData telem {};
        uint32_t flags = 0;
        int byte_index = 0;
// Ensure protocol hasn't expanded such that the warnings bits don't fit in a uint32_t
#if (getMaxLengthOfESC_WarningBits_t() + getMaxLengthOfESC_ExtendedWarningBits_t() > 4)
#error Size of PiccoloCAN warning bits exceeded TelemetryData flags size
#endif
        // Use protogen encoding to store bools into "flags" bitfield
        encodeESC_WarningBits_t((uint8_t*)&flags, &byte_index, &status.warnings);
        encodeESC_ExtendedWarningBits_t((uint8_t*)&flags, &byte_index, &status.warnings);

        telem.flags = flags;
        update_telem_data(addr, telem, AP_ESC_Telem_Backend::TelemetryType::FLAGS);
#endif // AP_EXTENDED_ESC_TELEM_ENABLED
        
        newTelemetry = true;
    } else if (decodeESC_FirmwarePacketStructure(&frame, &settings.firmware)) {
    } else if (decodeESC_AddressPacketStructure(&frame, &settings.address)) {
    } else if (decodeESC_EEPROMSettingsPacketStructure(&frame, &settings.eeprom)) {
    } else {
        result = false;
    }

    if (result) {
        reset_rx_timestamp();
    }

    return result;
}


/* Piccolo Glue Logic
 * The following functions are required by the auto-generated protogen code.
 */

//! \return the packet data pointer from the packet
uint8_t* getESCVelocityPacketData(void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (uint8_t*) frame->data;
}

//! \return the packet data pointer from the packet, const
const uint8_t* getESCVelocityPacketDataConst(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (const uint8_t*) frame->data;
}

//! Complete a packet after the data have been encoded
void finishESCVelocityPacket(void* pkt, int size, uint32_t packetID)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    if (size > AP_HAL::CANFrame::MaxDataLen) {
        size = AP_HAL::CANFrame::MaxDataLen;
    }

    frame->dlc = size;

    /* Encode the CAN ID
     * 0x07mm20dd
     * - 07 = ACTUATOR group ID
     * - mm = Message ID
     * - 20 = ESC actuator type
     * - dd = Device ID
     *
     * Note: The Device ID (lower 8 bits of the frame ID) will have to be inserted later
     */

    uint32_t id = (((uint8_t) PiccoloCAN_MessageGroup::ACTUATOR) << 24) |       // CAN Group ID
                  ((packetID & 0xFF) << 16) |                                       // Message ID
                  (((uint8_t) PiccoloCAN_ActuatorType::ESC) << 8);              // Actuator type

    // Extended frame format
    id |= AP_HAL::CANFrame::FlagEFF;

    frame->id = id;
}

//! \return the size of a packet from the packet header
int getESCVelocityPacketSize(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    return (int) frame->dlc;
}

//! \return the ID of a packet from the packet header
uint32_t getESCVelocityPacketID(const void* pkt)
{
    AP_HAL::CANFrame* frame = (AP_HAL::CANFrame*) pkt;

    // Extract the message ID field from the 29-bit ID
    return (uint32_t) ((frame->id >> 16) & 0xFF);
}

#endif // HAL_PICCOLO_CAN_ENABLE
