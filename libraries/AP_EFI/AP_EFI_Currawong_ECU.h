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
 * AP_EFI_Currawong_ECU.h
 *
 *      Author: Reilly Callaway
 */
 
#pragma once

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

#ifndef HAL_EFI_CURRAWONG_ECU_ENABLED
#define HAL_EFI_CURRAWONG_ECU_ENABLED HAL_MAX_CAN_PROTOCOL_DRIVERS && !HAL_MINIMIZE_FEATURES
#endif

#if HAL_EFI_CURRAWONG_ECU_ENABLED

#include <SRV_Channel/SRV_Channel.h>

class AP_EFI_Currawong_ECU : public AP_EFI_Backend {
public:
    AP_EFI_Currawong_ECU(AP_EFI &_frontend);
    
    void update() override;

    float getThrottle(void) { return m_throttle; }
    bool isNewThrottleCmd(void) { return m_newThrottleCmd; }

    static AP_EFI_Currawong_ECU* get_instance(void)
    {
        if (singleton == nullptr)
        {
            singleton = new AP_EFI_Currawong_ECU(*(AP_EFI().get_singleton()));
        }
        return singleton;
    }

private:
    bool handle_message(AP_HAL::CANFrame &frame);
    void updateThrottleCommand(SRV_Channel::Aux_servo_function_t source = SRV_Channel::k_throttle);

    static AP_EFI_Currawong_ECU *singleton;

    float m_throttle;
    bool m_newThrottleCmd;

    friend class AP_PiccoloCAN;
};

#endif // HAL_EFI_CURRAWONG_ECU_ENABLED

