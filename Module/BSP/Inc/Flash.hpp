#pragma once
#ifndef FLASH_H
#define FLASH_H

#include <cstring>
#include "main.h"

namespace Flash {

    enum Flash_Element_ID {
        Element_ID_BASE = 0,
        /*Write your flash element ID here*/
        Element_ID_GYRO,

        /*End of flash element ID*/
        Element_ID_MAX
    };

    __PACKED_STRUCT Flash_t {
        uint32_t bytes;
        uint8_t *address;
        uint8_t crc[2];
    };
    //Flash storaged as:
    //Head: 6 bytes
    //Data: n bytes

#define    FLASH_STORAGE_BASE_ADDRESS   0x080E0000 //Sector 7
#define    FLASH_STORAGE_SECTOR         FLASH_SECTOR_7 //Sector 7
#define    FLASH_STORAGE_SIZE           0x00020000 //128 Kbytes
#define    FLASH_STORAGE_CRC_FILED_SIZE 0x01  //8-bit CRC
#define    FALSH_STORAGE_HEAD_ALIEN_SIZE 12

    class cFlashCore {
    protected:
        Flash_t _flash_list[Element_ID_MAX]{};
        bool _rebuild_status[Element_ID_MAX]{};
    public:
        cFlashCore() {
            memset(_flash_list, 0, sizeof(_flash_list));
            memset(_rebuild_status, 0, sizeof(_rebuild_status));
            _rebuild_status[Element_ID_BASE] = true;
        }

        bool init();

        bool config_data(Flash_Element_ID element_id, uint8_t *data, uint32_t len);

        bool rebuild();

        bool get_element(Flash_Element_ID element_id, Flash_t *flash_element);

        bool flash_memcpy(Flash_Element_ID element_id, uint8_t *dst, bool en_crc = false);

        static void flash_memcpy(uint8_t *dst, const uint8_t *src, uint32_t len) {
            for (uint32_t i = 0; i < len; i++) {
                dst[i] = *(__IO uint8_t *) (src + i);
            }
        }
    };
}
extern Flash::cFlashCore flashCore;
#endif