#pragma once

// RP-Lidar Input Packets

#define RPLIDAR_CMD_SYNC_BYTE        0xA5
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD  0x80


#define RPLIDAR_ANS_SYNC_BYTE1       0xA5
#define RPLIDAR_ANS_SYNC_BYTE2       0x5A

#define RPLIDAR_ANS_PKTFLAG_LOOP     0x1

#define RPLIDAR_ANS_HEADER_SIZE_MASK        0x3FFFFFFF
#define RPLIDAR_ANS_HEADER_SUBTYPE_SHIFT    (30)

#if defined(_WIN32)
#pragma pack(1)
#endif

typedef struct _rplidar_cmd_packet_t {
    _u8 syncByte; //must be RPLIDAR_CMD_SYNC_BYTE
    _u8 cmd_flag;
    _u8 size;
    _u8 data[0];
} __attribute__((packed)) rplidar_cmd_packet_t;


typedef struct _rplidar_ans_header_t {
    _u8  syncByte1; // must be RPLIDAR_ANS_SYNC_BYTE1
    _u8  syncByte2; // must be RPLIDAR_ANS_SYNC_BYTE2
    _u32 size_q30_subtype; // see _u32 size:30; _u32 subType:2;
    _u8  type;
} __attribute__((packed)) rplidar_ans_header_t;

#if defined(_WIN32)
#pragma pack()
#endif


