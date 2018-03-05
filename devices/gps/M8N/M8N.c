/* M8N.c - HAL for gps device Ublox Neo-M8N
 *
 * Copyright (C) 2018 Arribada
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <string.h>
#include "M8N.h"
#include "syshal_gps.h"
#include "syshal_uart.h"
#include "debug.h"

// Private functions
static int syshal_gps_parse_rx_buffer_priv(UBX_Packet_t * packet);
static void syshal_gps_process_nav_status_priv(UBX_Packet_t * packet);
static void syshal_gps_process_nav_posllh_priv(UBX_Packet_t * packet);
static void syshal_gps_set_checksum_priv(UBX_Packet_t * packet);
static void syshal_gps_send_packet_priv(UBX_Packet_t * ubx_packet);

bool gps_locked = false; // Do we currently have a GPS lock
bool gps_locked_last = false;
uint32_t time_to_first_fix; // Time to first GPS fix in ms

struct
{
    uint32_t iTOW;   // GPS time of week of the navigation epoch
    int32_t  lon;    // Longitude
    int32_t  lat;    // Latitude
    int32_t  height; // Height above ellipsoid
    int32_t  hMSL;   // Height above mean sea level
} lastReadLocation; // Last known location

void syshal_gps_init(void)
{
    gps_locked = false;
    gps_locked_last = false;
    time_to_first_fix = 0;
    memset(&lastReadLocation, 0, sizeof(lastReadLocation)); // Clear structure
}

/**
 * @brief      GPS callback stub, should be overriden by the user application
 *
 * @param[in]  event  The event that occured
 */
__attribute__((weak)) void syshal_gps_callback(syshal_gps_event_t event)
{
    UNUSED(event);
    DEBUG_PR_WARN("%s Not implemented", __FUNCTION__);
}

/**
 * @brief      Get the most recent location from the GPS
 *
 * @param      iTOW       The tow
 * @param      longitude  The longitude
 * @param      latitude   The latitude
 * @param      height     The height
 *
 * @return     Returns true if the location received has been previously unread
 */
void syshal_gps_get_location(uint32_t * iTOW, int32_t * longitude, int32_t * latitude, int32_t * height)
{
    *iTOW = lastReadLocation.iTOW;
    *longitude = lastReadLocation.lon;
    *latitude = lastReadLocation.lat;
    *height = lastReadLocation.height;
}

uint32_t syshal_gps_time_till_first_fix(void)
{
    return time_to_first_fix;
}

void syshal_gps_shutdown(void)
{
    DEBUG_PR_TRACE("Shutdown GPS %s", __FUNCTION__);

    UBX_Packet_t ubx_packet;
    UBX_SET_PACKET_HEADER(&ubx_packet, UBX_MSG_CLASS_RXM, UBX_MSG_ID_RXM_PMREQ, sizeof(UBX_RXM_PMREQ2_t));
    UBX_PAYLOAD(&ubx_packet, UBX_RXM_PMREQ2)->version = UBX_RXM_PMREQ_VERSION;
    UBX_PAYLOAD(&ubx_packet, UBX_RXM_PMREQ2)->duration = 0; // Duration of the requested task, set to zero for infinite duration
    UBX_PAYLOAD(&ubx_packet, UBX_RXM_PMREQ2)->flags = UBX_RXM_PMREQ_FLAGS_BACKUP | UBX_RXM_PMREQ_FLAGS_FORCE; // The receiver goes into backup mode for a time period defined by duration
    UBX_PAYLOAD(&ubx_packet, UBX_RXM_PMREQ2)->wakeupSources = UBX_RXM_PMREQ_WAKEUP_UARTRX; // Configure pins to wakeup the receiver on a UART RX pin edge
    // No ACK is expected for this message
    syshal_gps_send_packet_priv(&ubx_packet);

    // We're sleeping so we don't have a gps lock
    if (gps_locked)
    {
        gps_locked = false;
        gps_locked_last = false;
        syshal_gps_callback(SYSHAL_GPS_EVENT_LOCK_LOST);
    }
}

void syshal_gps_wake_up(void)
{
    DEBUG_PR_TRACE("Wakeup GPS %s", __FUNCTION__);

    // We can send anything to wake the device
    uint8_t data = 0xAA;
    syshal_uart_transfer(GPS_UART, &data, 1);
}

void syshal_gps_tick(void)
{
    UBX_Packet_t ubx_packet;
    int error;

    error = syshal_gps_parse_rx_buffer_priv(&ubx_packet);

    if (GPS_UART_ERROR_CHECKSUM == error)
    {
        DEBUG_PR_TRACE("GPS Checksum error");
        return;
    }
    else if (GPS_UART_ERROR_MSG_TOO_BIG == error)
    {
        DEBUG_PR_TRACE("GPS Message too big");
        return;
    }
    else if (GPS_UART_ERROR_INSUFFICIENT_BYTES == error)
    {
        //DEBUG_PR_TRACE("GPS Uart insufficient bytes");
        return;
    }
    else if (GPS_UART_ERROR_MISSING_SYNC1 == error)
    {
        DEBUG_PR_TRACE("GPS missing Sync1");
        return;
    }
    else if (GPS_UART_ERROR_MISSING_SYNC2 == error)
    {
        DEBUG_PR_TRACE("GPS missing Sync2");
        return;
    }
    else if (GPS_UART_ERROR_MSG_PENDING == error)
    {
        //DEBUG_PR_TRACE("GPS message not fully received");
        return;
    }
    else if (GPS_UART_NO_ERROR != error)
    {
        DEBUG_PR_TRACE("GPS Generic comm error");
        return;
    }

    if (UBX_IS_MSG(&ubx_packet, UBX_MSG_CLASS_NAV, UBX_MSG_ID_NAV_STATUS))
        syshal_gps_process_nav_status_priv(&ubx_packet);
    else if (UBX_IS_MSG(&ubx_packet, UBX_MSG_CLASS_NAV, UBX_MSG_ID_NAV_POSLLH))
        syshal_gps_process_nav_posllh_priv(&ubx_packet);
    else
        DEBUG_PR_WARN("Unexpected GPS message class: (0x%02X) id: (0x%02X)", ubx_packet.msgClass, ubx_packet.msgId);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Private functions //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void syshal_gps_send_packet_priv(UBX_Packet_t * ubx_packet)
{
    syshal_gps_set_checksum_priv(ubx_packet);

    // The packet is not arranged contiguously in RAM, so we have to transmit
    // it using two passes i.e., header then payload + CRC
    syshal_uart_transfer(GPS_UART, (uint8_t *) ubx_packet, UBX_HEADER_LENGTH);
    syshal_uart_transfer(GPS_UART, ubx_packet->payloadAndCrc, ubx_packet->msgLength + UBX_CRC_LENGTH);
}

static void syshal_gps_process_nav_status_priv(UBX_Packet_t * packet)
{
    uint8_t gpsFix = UBX_PAYLOAD(packet, UBX_NAV_STATUS)->gpsFix;
    time_to_first_fix = UBX_PAYLOAD(packet, UBX_NAV_STATUS)->ttff;

    // Do we have a GPS fix?
    if ( (gpsFix > UBX_NAV_STATUS_GPSFIX_NOFIX) && (gpsFix <= UBX_NAV_STATUS_GPSFIX_TIME_ONLY) )
        gps_locked = true;
    else
        gps_locked = false;

    // Has our connection state changed from last time, if so notify the user application
    if (gps_locked_last != gps_locked)
    {
        if (gps_locked)
            syshal_gps_callback(SYSHAL_GPS_EVENT_LOCK_MADE);
        else
            syshal_gps_callback(SYSHAL_GPS_EVENT_LOCK_LOST);
    }

    gps_locked_last = gps_locked;
}

static void syshal_gps_process_nav_posllh_priv(UBX_Packet_t * packet)
{
    if (gps_locked)
    {
        lastReadLocation.iTOW = UBX_PAYLOAD(packet, UBX_NAV_POSLLH)->iTOW;
        lastReadLocation.lon = UBX_PAYLOAD(packet, UBX_NAV_POSLLH)->lon;
        lastReadLocation.lat = UBX_PAYLOAD(packet, UBX_NAV_POSLLH)->lat;
        lastReadLocation.height = UBX_PAYLOAD(packet, UBX_NAV_POSLLH)->height;
        lastReadLocation.hMSL = UBX_PAYLOAD(packet, UBX_NAV_POSLLH)->hMSL;

        syshal_gps_callback(SYSHAL_GPS_EVENT_NEW_DATA); // Notify the user application that there is new data pending
    }
}

static void syshal_gps_compute_checksum_priv(UBX_Packet_t * packet, uint8_t ck[2])
{
    ck[0] = ck[1] = 0;
    uint8_t * buffer = &packet->msgClass;

    /* Taken directly from Section 29 UBX Checksum of the u-blox8
     * receiver specification
     */
    for (unsigned int i = 0; i < 4; i++)
    {
        ck[0] = ck[0] + buffer[i];
        ck[1] = ck[1] + ck[0];
    }

    buffer = packet->payloadAndCrc;
    for (unsigned int i = 0; i < packet->msgLength; i++)
    {
        ck[0] = ck[0] + buffer[i];
        ck[1] = ck[1] + ck[0];
    }
}

void syshal_gps_set_checksum_priv(UBX_Packet_t * packet)
{
    uint8_t ck[2];

    //assert(packet->msgLength <= UBX_MAX_PACKET_LENGTH);
    syshal_gps_compute_checksum_priv(packet, ck);
    packet->payloadAndCrc[packet->msgLength]   = ck[0];
    packet->payloadAndCrc[packet->msgLength + 1] = ck[1];
}

// Return 0 on CRC match
int syshal_gps_check_checksum_priv(UBX_Packet_t * packet)
{
    uint8_t ck[2];

    //assert(packet->msgLength <= UBX_MAX_PACKET_LENGTH);
    syshal_gps_compute_checksum_priv(packet, ck);
    return (ck[0] == packet->payloadAndCrc[packet->msgLength] &&
            ck[1] == packet->payloadAndCrc[packet->msgLength + 1]) ? 0 : -1;
}

static int syshal_gps_parse_rx_buffer_priv(UBX_Packet_t * packet)
{

    // Discard everything up until first sync word
    uint32_t bytesInRxBuffer = syshal_uart_available(GPS_UART);

    // Check for minimum allowed message size
    if (bytesInRxBuffer < UBX_HEADER_AND_CRC_LENGTH)
        return GPS_UART_ERROR_INSUFFICIENT_BYTES;

    // Look for SYNC1 byte
    for (uint32_t i = 0; i < bytesInRxBuffer; ++i)
    {
        if (!syshal_uart_peek_at(GPS_UART, &packet->syncChars[0], 0))
            return GPS_UART_ERROR_INSUFFICIENT_BYTES;

        if (UBX_PACKET_SYNC_CHAR1 == packet->syncChars[0])
            goto label_sync_start;
        else
            syshal_uart_receive(GPS_UART, &packet->syncChars[0], 1); // remove this character
    }

    // No SYNC1 found
    return GPS_UART_ERROR_MISSING_SYNC1;

label_sync_start:

    // Get the next character and see if it is the expected SYNC2
    if (!syshal_uart_peek_at(GPS_UART, &packet->syncChars[1], 1))
        return GPS_UART_ERROR_INSUFFICIENT_BYTES;

    if (UBX_PACKET_SYNC_CHAR2 != packet->syncChars[1])
    {
        // Okay so SYNC1 is valid but SYNC2 is not
        // We should dispose of both to prevent the program locking
        uint8_t dumpBuffer[2];
        syshal_uart_receive(GPS_UART, &dumpBuffer[0], 2);
        return GPS_UART_ERROR_MISSING_SYNC2; // Invalid SYNC2
    }

    // Extract length field and check it is received fully
    uint8_t lengthLower, lengthUpper;

    if (!syshal_uart_peek_at(GPS_UART, &lengthLower, 4))
        return GPS_UART_ERROR_INSUFFICIENT_BYTES;

    if (!syshal_uart_peek_at(GPS_UART, &lengthUpper, 5))
        return GPS_UART_ERROR_INSUFFICIENT_BYTES;

    uint16_t payloadLength = (uint16_t)lengthLower | ((uint16_t)lengthUpper << 8);
    uint16_t totalLength = payloadLength + UBX_HEADER_AND_CRC_LENGTH;

    if (totalLength > UART_RX_BUF_SIZE)
    {
        uint8_t dumpBuffer;

        // Message is too big to store so throw it all away
        while (syshal_uart_available(GPS_UART) > 0)
            syshal_uart_receive(GPS_UART, &dumpBuffer, 1);

        return GPS_UART_ERROR_MSG_TOO_BIG;
    }

    // Check message is fully received and is in the RX buffer
    if (totalLength > bytesInRxBuffer)
        return GPS_UART_ERROR_MSG_PENDING;

    // Message is okay, lets grab the lot and remove it from the buffer
    uint8_t * buffer = (uint8_t *) packet;

    if (UBX_HEADER_LENGTH != syshal_uart_receive(GPS_UART, buffer, UBX_HEADER_LENGTH))
        return GPS_UART_ERROR_INSUFFICIENT_BYTES;

    buffer = packet->payloadAndCrc; // Now lets get the payload

    uint32_t totalToRead = payloadLength + UBX_CRC_LENGTH;
    if (totalToRead != syshal_uart_receive(GPS_UART, buffer, totalToRead))
        return GPS_UART_ERROR_INSUFFICIENT_BYTES;

    // Compute CRC and return
    if (syshal_gps_check_checksum_priv(packet) != 0)
        return GPS_UART_ERROR_CHECKSUM;

    return GPS_UART_NO_ERROR;
}