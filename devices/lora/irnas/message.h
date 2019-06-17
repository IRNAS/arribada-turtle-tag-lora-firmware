#ifndef _IRNAS_LORA_MESSAGE_H_
#define _IRNAS_LORA_MESSAGE_H_

#include <stdint.h>
#include <sys/types.h>

// Maximum number of TLVs inside a message.
#define MESSAGE_MAX_TLV_COUNT 3
// Maximum size of a value inside a message.
#define MESSAGE_MAX_VALUE_SIZE 64

/**
 * TLVs supported by the protocol.
 */
typedef enum {
    TLV_INIT = 1,
    TLV_GPS_LOCATION = 2,
} tlv_type_t;

/**
 * Contents of the GPS location TLV.
 */
typedef struct {
    uint32_t itow;    // GPS time of week of the navigation epoch
    int32_t  lon;     // Longitude
    int32_t  lat;     // Latitude
    int32_t  h_msl;   // Height above mean sea level
    uint32_t h_acc;   // Horizontal accuracy estimate
    uint32_t v_acc;   // Vertical accuracy estimate
} __attribute__((packed)) tlv_gps_location_t;

/**
 * Message operations result codes.
 */
typedef enum {
    MESSAGE_SUCCESS = 0,
    MESSAGE_ERROR_TOO_MANY_TLVS = -1,
    MESSAGE_ERROR_BUFFER_TOO_SMALL = -2,
    MESSAGE_ERROR_PARSE_ERROR = -3,
    MESSAGE_ERROR_TLV_NOT_FOUND = -4
} message_result_t;

/**
 * Representation of a TLV.
 */
typedef struct {
    uint8_t type;
    uint16_t length;
    uint8_t value[MESSAGE_MAX_VALUE_SIZE];
} tlv_t;

/**
 * Representation of a protocol message.
 */
typedef struct {
    size_t length;
    tlv_t tlv[MESSAGE_MAX_TLV_COUNT];
} message_t;

/**
 * Initializes a protocol message. This function should be called on any newly
 * created message to ensure that the underlying memory is properly initialized.
 *
 * @param message Message instance to initialize
 * @return Operation result code
 */
message_result_t message_init(message_t *message);

/**
 * Frees a protocol message.
 *
 * @param message Message instance to free
 */
void message_free(message_t *message);

/**
 * Parses a protocol message.
 *
 * @param message Destination message instance to parse into
 * @param data Raw data to parse
 * @param length Size of the data buffer
 * @return Operation result code
 */
message_result_t message_parse(message_t *message, const uint8_t *data, size_t length);

/**
 * Adds a raw TLV to a protocol message.
 *
 * @param message Destination message instance to add the TLV to
 * @param type TLV type
 * @param length TLV length
 * @param value TLV value
 * @return Operation result code
 */
message_result_t message_tlv_add(message_t *message, uint8_t type, uint16_t length, const uint8_t *value);

/**
 * Adds a GPS location TLV to a protocol message.
 */
message_result_t message_tlv_add_gps_location(message_t *message, const tlv_gps_location_t *loc);

/**
 * Finds the first TLV of a given type in a message and copies it.
 *
 * @param message Message instance to get the TLV from
 * @param type Type of TLV that should be returned
 * @param destination Destination buffer
 * @param length Length of the destination buffer
 * @return Operation result code
 */
message_result_t message_tlv_get(const message_t *message, uint8_t type, uint8_t *destination, size_t length);

/**
 * Finds the first GPS location TLV in a message and copies it.
 */
message_result_t message_tlv_get_gps_location(const message_t *message, tlv_gps_location_t *loc);

/**
 * Returns the size a message would take in its serialized form.
 *
 * @param message Message instance to calculate the size for
 */
size_t message_serialized_size(const message_t *message);

/**
 * Serializes a protocol message into a destination buffer.
 *
 * @param buffer Destination buffer
 * @param length Length of the destination buffer
 * @param message Message instance to serialize
 * @return Number of bytes written serialized to the buffer or error code
 */
ssize_t message_serialize(uint8_t *buffer, size_t length, const message_t *message);

/**
 * Prints a debug representation of a protocol message.
 *
 * @param message Protocol message to print
 */
void message_print(const message_t *message);

#endif
