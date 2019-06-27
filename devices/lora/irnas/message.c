#include "message.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static uint16_t htons(uint16_t v) {
    return (v >> 8) | (v << 8);
}

static uint32_t htonl(uint32_t v) {
    return htons(v >> 16) | (htons((uint16_t) v) << 16);
}
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
static uint16_t htons(uint16_t v) {
    return v;
}

static uint32_t htonl(uint32_t v) {
  return v;
}
#else
#error "Unknown byte order"
#endif

static uint16_t ntohs(uint16_t v) {
    return htons(v);
}

static uint32_t ntohl(uint32_t v) {
  return htonl(v);
}

message_result_t message_init(message_t *message)
{
    memset(message, 0, sizeof(message_t));
    return MESSAGE_SUCCESS;
}

void message_free(message_t *message)
{
    message_init(message);
}

message_result_t message_parse(message_t *message, const uint8_t *data, size_t length)
{
    message_init(message);

    size_t offset = 0;
    while (offset < length) {
        if (message->length >= MESSAGE_MAX_TLV_COUNT) {
            message_free(message);
            return MESSAGE_ERROR_TOO_MANY_TLVS;
        }

        size_t i = message->length;

        // Parse type.
        message->tlv[i].type = data[offset];
        offset += sizeof(uint8_t);

        if (offset + sizeof(uint16_t) > length) {
            message_free(message);
            return MESSAGE_ERROR_PARSE_ERROR;
        }

        // Parse length.
        memcpy(&message->tlv[i].length, &data[offset], sizeof(uint16_t));
        message->tlv[i].length = ntohs(message->tlv[i].length);
        offset += sizeof(uint16_t);

        if (offset + message->tlv[i].length > length) {
            message_free(message);
            return MESSAGE_ERROR_PARSE_ERROR;
        }

        if (message->tlv[i].length > MESSAGE_MAX_VALUE_SIZE) {
            message_free(message);
            return MESSAGE_ERROR_BUFFER_TOO_SMALL;
        }

        // Copy value.
        memcpy(message->tlv[i].value, &data[offset], message->tlv[i].length);
        offset += message->tlv[i].length;

        message->length++;
    }

    return MESSAGE_SUCCESS;
}

message_result_t message_tlv_add(message_t *message, uint8_t type, uint16_t length, const uint8_t *value)
{
    if (message->length >= MESSAGE_MAX_TLV_COUNT) {
        return MESSAGE_ERROR_TOO_MANY_TLVS;
    }
    if (length > MESSAGE_MAX_VALUE_SIZE) {
        return MESSAGE_ERROR_BUFFER_TOO_SMALL;
    }

    size_t i = message->length;
    message->tlv[i].type = type;
    message->tlv[i].length = length;
    memcpy(message->tlv[i].value, value, length);
    message->length++;

    return MESSAGE_SUCCESS;
}

message_result_t message_tlv_add_gps_location(message_t *message, const tlv_gps_location_t *loc)
{
    tlv_gps_location_t tmp;
    tmp.itow = htonl(loc->itow);
    tmp.lon = (int32_t) htonl((int32_t) loc->lon);
    tmp.lat = (int32_t) htonl((int32_t) loc->lat);
    tmp.h_msl = (int32_t) htonl((int32_t) loc->h_msl);
    tmp.h_acc = htonl(loc->h_acc);
    tmp.v_acc = htonl(loc->v_acc);
    return message_tlv_add(message, TLV_GPS_LOCATION, sizeof(tlv_gps_location_t), (uint8_t*) &tmp);
}

message_result_t message_tlv_get(const message_t *message, uint8_t type, uint8_t *destination, size_t length)
{
    for (size_t i = 0; i < message->length; i++) {
        if (message->tlv[i].type == type) {
            if (message->tlv[i].length > length) {
                return MESSAGE_ERROR_BUFFER_TOO_SMALL;
            }
            memcpy(destination, message->tlv[i].value, message->tlv[i].length);
            return MESSAGE_SUCCESS;
        }
    }

    return MESSAGE_ERROR_TLV_NOT_FOUND;
}

message_result_t message_tlv_get_gps_location(const message_t *message, tlv_gps_location_t *loc)
{
    message_result_t result = message_tlv_get(message, TLV_GPS_LOCATION, (uint8_t*) loc, sizeof(tlv_gps_location_t));
    if (result != MESSAGE_SUCCESS) {
        return result;
    }

    loc->itow = ntohl(loc->itow);
    loc->lon = (int32_t) ntohl((int32_t) loc->lon);
    loc->lat = (int32_t) ntohl((int32_t) loc->lat);
    loc->h_msl = (int32_t) ntohl((int32_t) loc->h_msl);
    loc->h_acc = ntohl(loc->h_acc);
    loc->v_acc = ntohl(loc->v_acc);

    return MESSAGE_SUCCESS;
}

size_t message_serialized_size(const message_t *message)
{
    size_t size = 0;
    for (size_t i = 0; i < message->length; i++) {
        // 1 byte type + 2 byte value + value length.
        size += sizeof(uint8_t) + sizeof(uint16_t) + message->tlv[i].length;
    }

    return size;
}

ssize_t message_serialize(uint8_t *buffer, size_t length, const message_t *message)
{
    size_t offset = 0;
    size_t remaining_buffer = length;
    for (size_t i = 0; i < message->length; i++) {
        size_t tlv_length = sizeof(uint8_t) + sizeof(uint16_t) + message->tlv[i].length;
        if (remaining_buffer < tlv_length) {
            return MESSAGE_ERROR_BUFFER_TOO_SMALL;
        }

        uint16_t hlength = htons(message->tlv[i].length);

        memcpy(buffer + offset, &message->tlv[i].type, sizeof(uint8_t));
        memcpy(buffer + offset + sizeof(uint8_t), &hlength, sizeof(uint16_t));
        memcpy(buffer + offset + sizeof(uint8_t) + sizeof(uint16_t), message->tlv[i].value, message->tlv[i].length);

        offset += tlv_length;
        remaining_buffer -= tlv_length;
    }

    return offset;
}

void message_print(const message_t *message)
{
    printf("<Message tlvs(%u)=[", (unsigned int) message->length);
    for (size_t i = 0; i < message->length; i++) {
        const uint8_t *data = message->tlv[i].value;
        size_t data_length = message->tlv[i].length;

        printf("{%u, \"", message->tlv[i].type);
        for (size_t j = 0; j < data_length; j++) {
            printf("%02X%s", data[j], (j < data_length - 1) ? " " : "");
        }
        printf("\"}%s", (i < message->length - 1) ? "," : "");
    }
    printf("]>");
}
