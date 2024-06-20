#ifndef MIN_ID_H
#define MIN_ID_H

#define MIN_ID_GET_GPIO                     0x00
#define MIN_ID_SET_GPIO                     0x01
#define MIN_ID_UPDATE_ALL                   0x02
#define MIN_ID_PING                         0x03
#define MIN_ID_BUTTON_ISR                   0x04
#define MIN_ID_SLAVE_RESET                  0x05
#define MIN_ID_RESET						0x06
#define MIN_ID_OTA_UPDATE_START             0x07
#define MIN_ID_OTA_UPDATE_TRANSFER          0x08
#define MIN_ID_OTA_UPDATE_END               0x09
#define MIN_ID_OTA_ACK                      0x0A
#define MIN_ID_OTA_FAILED                   0x0B
#define MIN_ID_BUFFER_FULL                  0x0C
#define MIN_ID_MASTER_OTA_BEGIN             0x0D
#define MIN_ID_MASTER_OTA_END               0x0E
#define MIN_ID_RS485_FORWARD                0x0F
#define MIN_ID_RS232_FORWARD                0x10
#define MIN_ID_JIG_FORWARD                  0x11
#define MIN_ID_SET_VOLUME_CALL              17
#define MIN_ID_CHANGE_VOLUME_CALL           18
#define MIN_ID_MUTE_VOLUME_CALL             19
#define MIN_ID_MUTE_MIC_CALL                20

#define MIN_ID_UPDATE_UI                    28
#define MIN_ID_REMOTE_PLAY_MUSIC            29
#define MIN_ID_LOCAL_PLAY_MIC               30
#define MIN_ID_MASTER_SELECT_ROOM           32
#define MIN_ID_MASTER_SELECT_ALL_ROOM       33

#define MIN_ID_SELECT_STORAGE_TYPE          43
#define MIN_ID_SET_UNIX_TIMESTAMP           44
#define MIN_ID_CALL_PRESSED                 45    
#define MIN_ID_RESET_LCD                    46

#define MIN_ID_PLAY_STOP_SONG	            36
#define MIN_ID_PREVIOUS_SONG	            37
#define MIN_ID_NEXT_SONG	                38
#define MIN_ID_SELECT_SONG	                39
#define MIN_ID_SELECT_LOOP	                40
#define MIN_ID_SELECT_FOLDER                41
#define MIN_ID_PREVIOUS_FOLDER              42
#define MIN_ID_MUTE_MUSIC_VOLUME            49
#define MIN_ID_CHANGE_MUSIC_VOLUME          48
#define MIN_ID_FILE_SCROLL_LEFT             50
#define MIN_ID_FILE_SCROLL_RIGHT            51
#define MIN_ID_IO_BUTTON_PRESS              52
#define MIN_ID_NW_GET_IP                    53
#define MIN_ID_FORWARD                      54
#define MIN_ID_NEXTION_LCD                  55
#define MIN_ID_FORWARD_FM_MSG_TO_MCU        56
#define MIN_ID_FORWARD_DBG_MSG              57
#define MIN_ID_REQUEST_RESET_REASON         58
#define MIN_ID_CTRL_EXT_GPIO                59

#endif /* MIN_ID_H */
