#ifndef VIC_DEFINES_H
#define VIC_DEFINES_H

typedef struct  __attribute__((packed))
{
    uint16_t header;
    uint16_t message;
    uint16_t footer;
} vic_message_frame_t;


#endif //VIC_DEFINES_H