#include "Flash.hpp"
#include "CRC8.h"

using namespace Flash;
cFlashCore flashCore;

bool cFlashCore::init() {
    uint8_t flash_struct_alien[FALSH_STORAGE_HEAD_ALIEN_SIZE];
    Flash_t flash_struct{};
    bool fault_flag = false;
    auto *address = (uint8_t *) FLASH_STORAGE_BASE_ADDRESS;

    for (auto &data: _flash_list) {
        // When error happened, stop search
        if (fault_flag) {
            data.address = nullptr;
            continue;
        }

        // Read head
        for (uint32_t i = 0; i < FALSH_STORAGE_HEAD_ALIEN_SIZE; i++) {
            ((uint8_t *) (&flash_struct_alien))[i] = *(__IO uint8_t *) (address + i);
        }
        memcpy(&flash_struct, flash_struct_alien, sizeof(Flash_t));
        // Check head
        if (flash_struct.crc[1] == cal_crc8_table(address, sizeof(Flash_t) - FLASH_STORAGE_CRC_FILED_SIZE)) {
            memcpy(&data, &flash_struct, sizeof(flash_struct));
            address += sizeof(flash_struct) + data.bytes;
        } else {
            data.address = nullptr;
            fault_flag = true;
        }
    }

    return fault_flag;
}

bool cFlashCore::config_data(Flash_Element_ID element_id, uint8_t *data, uint32_t len) {
    //Check ID
    if ((element_id == Element_ID_BASE) || (element_id >= Element_ID_MAX)) {
        return false;
    }
    _flash_list[element_id].address = data;
    _flash_list[element_id].bytes = len;
    _flash_list[element_id].crc[0] = cal_crc8_table(data, len);
    _rebuild_status[element_id] = true;
    return true;
}

bool cFlashCore::rebuild() {
    uint32_t error_msg;
    auto *address = (uint8_t *) FLASH_STORAGE_BASE_ADDRESS;
    FLASH_EraseInitTypeDef EraseInitStruct{.TypeErase=FLASH_TYPEERASE_SECTORS,.Banks=FLASH_BANK_1, .Sector=FLASH_STORAGE_SECTOR, .NbSectors=1, .VoltageRange=FLASH_VOLTAGE_RANGE_3};
    uint32_t en_address = FLASH_STORAGE_BASE_ADDRESS + Element_ID_MAX * FALSH_STORAGE_HEAD_ALIEN_SIZE;

    /*Check if all data elements are ready*/
    for (uint32_t i = 0; i < Element_ID_MAX; i++) {
        if (!_rebuild_status[i]) {
            return false;
        }
        en_address += _flash_list[i].bytes;
    }
    /*Check if data is larger than flash*/
    if (en_address > FLASH_STORAGE_BASE_ADDRESS + FLASH_STORAGE_SIZE) {
        return false;
    }

    _flash_list[0].address = (uint8_t *) FLASH_STORAGE_BASE_ADDRESS;
    _flash_list[0].bytes = 0x00;
    _flash_list[0].crc[0] = 0x00;

    /*Start program flash*/
    HAL_FLASH_Unlock();
    /*Erase Flash*/
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &error_msg) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }
    for (auto &data: _flash_list) {
        //Write data to flash
        uint8_t num_words = data.bytes / 4;
        uint8_t num_btyes = data.bytes % 4;
        uint32_t remain_bytes = 0;
        //Write as word
        for (uint32_t len = 0; len < num_words; len++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                  (uint32_t) (address + FALSH_STORAGE_HEAD_ALIEN_SIZE + 4 * len),
                                  (uint64_t) data.address[4 * len]) != HAL_OK) {
                HAL_FLASH_Lock();
                return false;
            }
        }
        //Write remaining bytes
        if (num_btyes != 0) {
            memcpy(&remain_bytes, &data.address[4 * num_words], num_btyes);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                  (uint32_t) (address + FALSH_STORAGE_HEAD_ALIEN_SIZE + 4 * num_words),
                                  (uint64_t) remain_bytes) != HAL_OK) {
                HAL_FLASH_Lock();
                return false;
            }
        }
        //Reset head
        data.address = address + FALSH_STORAGE_HEAD_ALIEN_SIZE;
        data.crc[1] = cal_crc8_table((uint8_t *) &data, sizeof(Flash_t) - FLASH_STORAGE_CRC_FILED_SIZE);

        //Write head to flash
        uint8_t head_alien[FALSH_STORAGE_HEAD_ALIEN_SIZE];
        memcpy(head_alien, &data, sizeof(Flash_t));
        for (uint8_t len = 0; len < FALSH_STORAGE_HEAD_ALIEN_SIZE / 4; len++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t) (address + 4 * len),
                                  (uint64_t) head_alien[4 * len]) != HAL_OK) {
                HAL_FLASH_Lock();
                return false;
            }
        }

        //Recursive new address
        address += FALSH_STORAGE_HEAD_ALIEN_SIZE + data.bytes;
    }
    HAL_FLASH_Lock();
    return true;
}

bool cFlashCore::get_element(Flash_Element_ID element_id, Flash_t *flash_element) {
    if (element_id >= Element_ID_MAX) {
        return false;
    }
    if (_flash_list[element_id].address == nullptr) {
        return false;
    }
    memcpy(flash_element, &_flash_list[element_id], sizeof(Flash_t));
    return true;
}

bool cFlashCore::flash_memcpy(Flash_Element_ID element_id, uint8_t *dst, bool en_crc) {
    if (element_id >= Element_ID_MAX) {
        return false;
    }

    if (_flash_list[element_id].address == nullptr) {
        return false;
    }

    flash_memcpy(dst, _flash_list[element_id].address, _flash_list[element_id].bytes);

    if (en_crc) {
        if (cal_crc8_table(dst, _flash_list[element_id].bytes) != _flash_list[element_id].crc[0]) {
            return false;
        }
    }

    return true;
}