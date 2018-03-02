/* fs.c - Flash File system
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
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "fs_priv.h"
#include "fs.h"
#include "syshal_flash.h"

/* Constants */

/* Macros */

#define MIN(x, y)  (x) < (y) ? (x) : (y)

/* Types */

/* Static Data */

/* List of file system devices */
static fs_priv_t fs_priv_list[FS_PRIV_MAX_DEVICES];

/* List of file handles (shared across all devices) */
static fs_priv_handle_t fs_priv_handle_list[FS_PRIV_MAX_HANDLES];

/* Static Functions */


static inline uint8_t get_user_flags(fs_priv_t *fs_priv, uint8_t sector)
{
    return fs_priv->alloc_unit_list[sector].file_info.file_flags.user_flags;
}

static inline uint8_t get_mode_flags(fs_priv_t *fs_priv, uint8_t sector)
{
    return fs_priv->alloc_unit_list[sector].file_info.file_flags.mode_flags;
}

static inline uint8_t get_file_protect(fs_priv_t *fs_priv, uint8_t sector)
{
    return fs_priv->alloc_unit_list[sector].file_info.file_protect;
}

static inline uint32_t get_alloc_counter(fs_priv_t *fs_priv, uint8_t sector)
{
    return fs_priv->alloc_unit_list[sector].alloc_counter;
}

static inline uint8_t get_file_id(fs_priv_t *fs_priv, uint8_t sector)
{
    return fs_priv->alloc_unit_list[sector].file_info.file_id;
}

static inline bool is_last_allocation_unit(fs_priv_t *fs_priv, uint8_t sector)
{
    return (fs_priv->alloc_unit_list[sector].file_info.next_allocation_unit ==
            (uint8_t)FS_PRIV_NOT_ALLOCATED);
}

static inline uint8_t next_allocation_unit(fs_priv_t *fs_priv, uint8_t sector)
{
    return fs_priv->alloc_unit_list[sector].file_info.next_allocation_unit;
}

/*! \brief Initialize private file system device structure.
 *
 * Scan through each sector and read the allocation unit
 * header into our local structure to prevent having to
 * read the flash device each time.
 *
 * \param device[in] the device instance number
 * \return \ref FS_ERROR_FLASH_MEDIA if a flash read failed
 * \return \ref FS_ERROR_BAD_DEVICE if the device number is bad
 * \return \ref FS_NO_ERROR on success
 */
static int init_fs_priv(uint32_t device)
{
    fs_priv_t *fs_priv;

    /* This device parameter is used as an index into the file
     * system device list.  We have to range check it before
     * proceeding to avoid bad stuff happening.
     */
    if (device >= FS_PRIV_MAX_DEVICES)
        return FS_ERROR_BAD_DEVICE;

    fs_priv = &fs_priv_list[device];
    fs_priv->device = device;  /* Keep a copy of the device index */

    /* Iterate through each sector and read the allocation unit header into
     * our file system device structure.
     */
    for (uint8_t sector = 0; sector < FS_PRIV_MAX_SECTORS; sector++)
    {
        if (syshal_flash_read(device, &fs_priv->alloc_unit_list[sector],
                FS_PRIV_SECTOR_ADDR(sector), sizeof(fs_priv_alloc_unit_header_t)))
            return FS_ERROR_FLASH_MEDIA;
    }

    /* TODO: we should probably implement some kind of file system
     * validation check here to avoid using a corrupt file system.
     */
    return FS_NO_ERROR;
}

/*! \brief Return the number of bytes cached on the file handle.
 *
 * \param fs_priv_handle[in] pointer to private flash handle.
 * \return number of bytes cached which shall be limited to the cache size.
 */
static inline uint16_t cached_bytes(fs_priv_handle_t *fs_priv_handle)
{
    return (fs_priv_handle->curr_data_offset - fs_priv_handle->last_data_offset);
}

/*! \brief Ascertain the number of bytes free in the allocation unit.
 *
 * \param fs_priv_handle[in] pointer to private flash handle.
 * \return integer number of bytes that may still be written.
 */static inline uint32_t remaining_bytes(fs_priv_handle_t *fs_priv_handle)
{
    return (FS_PRIV_USABLE_SIZE - fs_priv_handle->curr_data_offset);
}

/*! \brief Find a free allocation unit (sector) in the file system.
 *
 * The \ref fs_priv_file_info_t.file_id field is used to determine whether a
 * sector is allocated or not, so check each sector until we find one whose
 * \ref fs_priv_file_info_t.file_id is set to \ref FS_PRIV_NOT_ALLOCATED.
 *
 * \param fs_priv[in] a pointer to a file system device structure that
 *        we want to search
 * \return \ref FS_PRIV_NOT_ALLOCATED if no free sector found
 * \return sector number if a free sector was found
 */
static uint8_t find_free_allocation_unit(fs_priv_t *fs_priv)
{
    uint8_t min_allocation_counter = (uint8_t)FS_PRIV_NOT_ALLOCATED;
    uint8_t free_sector = (uint8_t)FS_PRIV_NOT_ALLOCATED;

    /* In the worst case, we have to check every sector on the disk to
     * find a free sector.
     */
    for (uint8_t sector = 0; sector < FS_PRIV_MAX_SECTORS; sector++)
    {
        /* Consider only unallocated sectors */
        if ((uint8_t)FS_PRIV_NOT_ALLOCATED == get_file_id(fs_priv, sector))
        {
            /* Special case for unformatted sector */
            if ((uint32_t)FS_PRIV_NOT_ALLOCATED == get_alloc_counter(fs_priv, sector))
            {
                /* Choose this sector since it has never been used */
                free_sector = sector;
                break;
            }
            else if (get_alloc_counter(fs_priv, sector) < min_allocation_counter)
            {
                /* This is now the least used sector */
                min_allocation_counter = fs_priv->alloc_unit_list[sector].alloc_counter;
                free_sector = sector;
            }
        }
    }

    return free_sector;
}

/*! \brief Allocate a file handle.
 *
 * The first free handle is found in \ref fs_priv_handle_list and
 * the \ref fs_priv_handle_t pointer shall be populated with it on success.
 *
 * \param fs_priv[in] a pointer to a file system device that we want to associate
 *        the file handle with
 * \param handle[out] a pointer to a private file handle pointer.  This shall be
 *        set to point to the free handle found.
 * \return \ref FS_ERROR_NO_FREE_HANDLE if all handles are in use.
 * \return \ref FS_NO_ERROR on success.
 */
static int allocate_handle(fs_priv_t *fs_priv, fs_priv_handle_t **handle)
{
    for (uint8_t i = 0; i < FS_PRIV_MAX_HANDLES; i++)
    {
        if (NULL == fs_priv_handle_list[i].fs_priv)
        {
            *handle = &fs_priv_handle_list[i];
            fs_priv_handle_list[i].fs_priv = fs_priv;

            return FS_NO_ERROR;
        }
    }

    return FS_ERROR_NO_FREE_HANDLE;
}

/*! \brief Free a file handle.
 *
 * The handle is freed by setting its \ref fs_priv_handle_t.fs_priv
 * member to NULL.
 *
 * \param handle[in] a pointer to a private file handle to be freed.
 */
static void free_handle(fs_priv_handle_t *handle)
{
    handle->fs_priv = NULL;
}

/*! \brief Check a file's protection bits (see \ref
 *  fs_priv_file_info_t.file_protect) to see if the file
 *  is write protected or not.
 *
 * The file protection scheme counts the number of set bits
 * to determine if a file is protected or not.  If there
 * are zero or an even number of bits, the file is not protected.
 * If there are an odd number of bits, the file is protected.
 *
 * \param protection_bits[in] taken from \ref
 *  fs_priv_file_info_t.file_protect
 * \return true if the file is protected, false otherwise
 */
static bool is_protected(uint8_t protection_bits)
{
    uint8_t count_bits;
    for (count_bits = 0; protection_bits; count_bits++)
        protection_bits &= protection_bits - 1; /* Clear LSB */

    /* Odd number of bits means the file is protected */
    return (count_bits & 1) ? true : false;
}

/*! \brief Set a file's protection bits (see \ref
 *  fs_priv_file_info_t.file_protect).
 *
 * If the protection state is to be modified then the
 * number of set bits shall be modified.  Caution
 * should be taken to ensure that the value is not
 * already zero since this results in no change to
 * the protection bits i.e., a file will remain
 * permanently unprotected until erased in this
 * situation.
 *
 * \param protected[in] the boolean state that we wish to apply
 *        to the protected bits
 * \param protected_bits[in] current file protection bits
 *        value as per \ref fs_priv_file_info_t.file_protect
 * \return new value of the file's protection bits which
 *         shall only be different if the file's protection
 *         state changed
 */
static uint8_t set_protected(bool protected, uint8_t protected_bits)
{
    /* Only update protected bits if the current bits do not
     * match the required protection state
     */
    if (protected != is_protected(protected_bits))
        protected_bits &= protected_bits - 1; /* Clear LSB */

    /* Return new protected bits */
    return protected_bits;
}

/*! \brief Find the root sector in a chain of linked
 *  allocation units on the file system for a given file.
 *
 * Each allocation unit has a \ref fs_priv_file_info_t.next_allocation_unit
 * pointer which may be used to form an order chain of sectors associated
 * with a given file.  This routine shall scan all sectors associated
 * with a file and determine the root sector i.e., the sector which
 * has no parent node.
 *
 * \param fs_priv[in] a pointer to the private file system structure.
 * \param file_id[in] unique file identifier
 * \return sector[in] number of the root sector on success
 * \return \ref FS_PRIV_NOT_ALLOCATED if the file was not found
 */
static uint8_t find_file_root(fs_priv_t *fs_priv, uint8_t file_id)
{
    uint8_t root = (uint8_t)FS_PRIV_NOT_ALLOCATED;
    uint8_t parent[FS_PRIV_MAX_SECTORS];

    /* Do not allow FS_PRIV_NOT_ALLOCATED as file_id */
    if ((uint8_t)FS_PRIV_NOT_ALLOCATED == file_id)
        return FS_PRIV_NOT_ALLOCATED;

    /* Reset parent list to known values */
    memset(parent, (uint8_t)FS_PRIV_NOT_ALLOCATED, sizeof(parent));

    /* Scan all sectors and build a list of parent nodes for each
     * sector allocated against the specified file_id
     */
    for (uint8_t sector = 0; sector < FS_PRIV_MAX_SECTORS; sector++)
    {
        /* Filter by file_id */
        if (file_id == get_file_id(fs_priv, sector))
        {
            if (!is_last_allocation_unit(fs_priv, sector))
                parent[next_allocation_unit(fs_priv, sector)] = sector;
            /* Arbitrarily choose first found sector as the candidate root node */
            if (root == (uint8_t)FS_PRIV_NOT_ALLOCATED)
                root = sector;
        }
    }

    /* Start with candidate root sector and walk all the parent nodes until we terminate */
    while (root != (uint8_t)FS_PRIV_NOT_ALLOCATED)
    {
        /* Does this node have a parent?  If not then it is the root node */
        if (parent[root] == (uint8_t)FS_PRIV_NOT_ALLOCATED)
            break;
        else
            root = parent[root]; /* Try next one in chain */
    }

    /* Return the root node which may be FS_PRIV_NOT_ALLOCATED if no file was found */
    return root;
}

/*! \brief Check consistency of file open mode versus file's stored flags.
 *
 * Certain file open modes are erroneous e.g., creating a file whose
 * file_id already exists, opening a file for writing that has been
 * protected.  This routine traps an such errors.
 *
 * \param fs_priv[in] a pointer to the private file system structure.
 * \param root[in] the root sector number containing the file's properties.
 * \param mode[in] desired mode requested for opening the file.
 * \return \ref FS_NO_ERROR if all checks pass
 * \return \ref FS_ERROR_FILE_NOT_FOUND if the root node was not found
 * \return \ref FS_ERROR_WRITE_PROTECTED if the file is write protected
 * \return \ref FS_ERROR_FILE_ALREADY_EXISTS if the file already exists
 */
static int check_file_flags(fs_priv_t *fs_priv, uint8_t root, fs_mode_t mode)
{
    if ((uint8_t)FS_PRIV_NOT_ALLOCATED == root)
    {
        /* File does not exist so unless this is a create request then
         * return an error.
         */
        if ((mode & FS_FILE_CREATE) == 0)
            return FS_ERROR_FILE_NOT_FOUND;
    }
    else
    {
        /* Don't allow the file to be created since it already exists */
        if (mode & FS_FILE_CREATE)
            return FS_ERROR_FILE_ALREADY_EXISTS;

        /* If opened as writeable then make sure file is not protected */
        const uint8_t protection_bits = fs_priv->alloc_unit_list[root].file_info.file_protect;
        if ((mode & FS_FILE_WRITEABLE) && is_protected(protection_bits))
            return FS_ERROR_FILE_PROTECTED;
    }

    return FS_NO_ERROR;
}

/*! \brief Scans to find next available write position in a sector.
 *
 * Before writing to a sector of a file, it is necessary to find
 * the last known write position and also the next available session
 * write offset, so the new write position can be tracked.
 *
 * \param fs_priv[in] a pointer to the private file system structure.
 * \param sector[in] the sector number to check.
 * \param data_offset[out] pointer to data offset relative to start of sector
 *        which shall be used to store the current write offset
 * \return session write offset e.g., 0 means the first session entry.
 * \return \ref FS_PRIV_NOT_ALLOCATED if no session entry is free.
 */
static uint8_t find_next_session_offset(fs_priv_t *fs_priv, uint8_t sector, uint32_t *data_offset)
{
    uint32_t write_offset = (uint8_t)FS_PRIV_NOT_ALLOCATED;
    uint32_t write_offsets[FS_PRIV_NUM_WRITE_SESSIONS];

    /* Read all the session offsets from flash */
    syshal_flash_read(fs_priv->device, (void *)write_offsets,
            FS_PRIV_SECTOR_ADDR(sector) + FS_PRIV_SESSION_OFFSET,
            sizeof(write_offsets));

    /* Scan session offsets to find first free entry.  If all entries
     * are already used then no further writes can be done and
     * FS_PRIV_NOT_ALLOCATED shall be returned.
     */
    for (uint8_t i = 0; i < FS_PRIV_NUM_WRITE_SESSIONS; i++)
    {
        if ((uint32_t)FS_PRIV_NOT_ALLOCATED == write_offsets[i])
        {
            if (i == 0)
                *data_offset = 0; /* None yet assigned */

            /* Next available write offset has been found */
            write_offset = i;
            break;
        }

        /* Set last known write offset */
        *data_offset = write_offsets[i];
    }

    return write_offset;
}

/*! \brief Find the last allocation unit in the chain of sectors
 *  for a given file.
 *
 * Simply start from the root sector and iterate until we reach
 * a sector whose \ref fs_priv_file_info_t.next_allocation_unit
 * is set to \ref FS_PRIV_NOT_ALLOCATED.
 *
 * \param fs_priv[in] a pointer to the private file system structure.
 * \param root[in] the root sector number to start from.
 * \return sector offset for the last sector of the file.
 */
static uint8_t find_last_allocation_unit(fs_priv_t *fs_priv, uint8_t root)
{
    /* Loop through until we reach the end of the file chain */
    while (root != (uint8_t)FS_PRIV_NOT_ALLOCATED)
    {
        if (next_allocation_unit(fs_priv, root) != (uint8_t)FS_PRIV_NOT_ALLOCATED)
            root = next_allocation_unit(fs_priv, root);
        else
            break;
    }

    return root;
}

/*! \brief Find the last allocation unit and also the last known
 * write position in that allocation unit.
 *
 * See \ref find_last_allocation_unit and \ref find_next_session_offset.
 *
 * \param fs_priv[in] a pointer to the private file system structure.
 * \param root[in] the root sector number to start from.
 * \param last_alloc_unit[out] pointer for storing the sector offset of
 *         the last allocation unit in the file chain.
 * \param data_offset pointer[out] for storing the last known write position
 *        in the last sector of the file.
 * \return session write offset e.g., 0 means the first session entry.
 * \return \ref FS_PRIV_NOT_ALLOCATED if no session entry is free.
 */
static uint8_t find_eof(fs_priv_t *fs_priv, uint8_t root, uint8_t *last_alloc_unit, uint32_t *data_offset)
{
    *last_alloc_unit = find_last_allocation_unit(fs_priv, root);
    return find_next_session_offset(fs_priv, root, data_offset);
}

/*! \brief Determines if a handle is in the end of a file condition.
 *
 * When a file is in write mode, this condition is only met if all
 * bytes in the page cache have been flushed to flash.
 *
 * When a file id in read mode then the condition is only met if the
 * current data offset is pointing to the last byte of the last sector
 * in the file chain.
 *
 * \param fs_priv_handle[in] pointer to private file handle
 * \return true if the handle is EOF, false otherwise
 */
static bool is_eof(fs_priv_handle_t *fs_priv_handle)
{
    fs_priv_t *fs_priv = fs_priv_handle->fs_priv;

    return ((fs_priv_handle->last_data_offset == fs_priv_handle->curr_data_offset) &&
            next_allocation_unit(fs_priv, fs_priv_handle->curr_allocation_unit) ==
                    (uint8_t)FS_PRIV_NOT_ALLOCATED);
}

/*! \brief Erase allocation unit in flash.
 *
 * This will physically erase the associated sector on flash media
 * and also update the allocation counter for the allocation unit.
 * Both the alloction unit header stored locally and stored in
 * flash are synchronized.
 *
 * \param fs_priv[in] a pointer to the private file system structure.
 * \param sector the sector number to erase.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA on error.
 */
static int erase_allocation_unit(fs_priv_t *fs_priv, uint8_t sector)
{
    /* Read existing allocation counter and increment for next allocation */
    uint32_t new_alloc_counter = fs_priv->alloc_unit_list[sector].alloc_counter + 1;

    /* Erase the entire sector (should be all FF) */
    if (syshal_flash_erase(fs_priv->device, FS_PRIV_SECTOR_ADDR(sector), FS_PRIV_SECTOR_SIZE))
        return FS_ERROR_FLASH_MEDIA;

    /* Reset local copy of allocation unit header */
    memset(&fs_priv->alloc_unit_list[sector], 0xFF, sizeof(fs_priv->alloc_unit_list[sector]));

    /* Set allocation counter locally */
    fs_priv->alloc_unit_list[sector].alloc_counter = new_alloc_counter;

    /* Write only the allocation counter to flash */
    if (syshal_flash_write(fs_priv->device, &new_alloc_counter,
            FS_PRIV_SECTOR_ADDR(sector) + FS_PRIV_ALLOC_COUNTER_OFFSET,
            sizeof(uint32_t)))
        return FS_ERROR_FLASH_MEDIA;

    return FS_NO_ERROR;
}

/*! \brief Write cached file data to flash memory.
 *
 * \param fs_priv_handle[in] pointer to private flash handle.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA if a flash write failed.
 */
static int flush_page_cache(fs_priv_handle_t *fs_priv_handle)
{
    uint32_t size, address;

    /* Compute number of bytes in page cache; note that the cache fill policy
     * means that we can never exceed the next page boundary, so we don't need to
     * worry about crossing a page boundary.
     */
    size = cached_bytes(fs_priv_handle);

    if (size > 0)
    {
        /* Physical address calculation */
        address = FS_PRIV_SECTOR_ADDR(fs_priv_handle->curr_allocation_unit) +
                FS_PRIV_ALLOC_UNIT_SIZE + fs_priv_handle->last_data_offset;

        /* Write cached data to flash */
        if (syshal_flash_write(fs_priv_handle->fs_priv->device, fs_priv_handle->page_cache,
                address, size))
            return FS_ERROR_FLASH_MEDIA;

        /* Mark the cache as empty and advance the pointer */
        fs_priv_handle->last_data_offset = fs_priv_handle->curr_data_offset;
    }

    return FS_NO_ERROR;
}

/*! \brief Write current session write offset to flash memory.
 *
 * \param fs_priv_handle[in] pointer to private flash handle.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA if a flash write failed.
 */
static int update_session_offset(fs_priv_handle_t *fs_priv_handle)
{
    uint32_t address;

    /* Compute physical address for next write offset */
    address = FS_PRIV_SECTOR_ADDR(fs_priv_handle->curr_allocation_unit) +
            FS_PRIV_SESSION_OFFSET +
            (sizeof(uint32_t) * fs_priv_handle->curr_session_offset);

    /* Write the new offset into the allocation unit */
    if (syshal_flash_write(fs_priv_handle->fs_priv->device,
            &fs_priv_handle->last_data_offset,
            address, sizeof(uint32_t)))
        return FS_ERROR_FLASH_MEDIA;

    /* Set next available write offset */
    fs_priv_handle->curr_session_offset++;
    if (fs_priv_handle->curr_session_offset >= FS_PRIV_NUM_WRITE_SESSIONS)
    {
        /* No further sessions free */
        fs_priv_handle->curr_session_offset = (uint8_t)FS_PRIV_NOT_ALLOCATED;
    }

    return FS_NO_ERROR;
}

/*! \brief Flush file handle.
 *
 * Forces the writing of any cached data and the session write
 * offset to flash memory.
 *
 * \param fs_priv_handle[in] pointer to private flash handle.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA if a flash write failed.
 * \return \ref FS_ERROR_FILESYSTEM_FULL if no session is free.
 */
static int flush_handle(fs_priv_handle_t *fs_priv_handle)
{
    int ret;

    /* Don't allow flush if a session write offset is not available */
    if ((uint8_t)FS_PRIV_NOT_ALLOCATED == fs_priv_handle->curr_session_offset)
        return FS_ERROR_FILESYSTEM_FULL;

    /* Flush any bytes in the page cache */
    ret = flush_page_cache(fs_priv_handle);
    if (ret)
        return ret;

    /* Set new write offset */
    return update_session_offset(fs_priv_handle);
}

/*! \brief Allocate new sector to a file.
 *
 * When creating or writing to a file it will be necessary to
 * sometimes allocate a new sector.  This involves find a free sector
 * and then updating the file allocation table to reflect that
 * a new sector has been chained to the end of the file.  Note
 * that any file allocation data is written to flash immediately to
 * ensure integrity of the file allocation information.
 *
 * \param fs_priv_handle[in] pointer to private flash handle for which we
 * want to allocate a new sector for.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA if a flash write failed.
 */
static int allocate_new_sector_to_file(fs_priv_handle_t *fs_priv_handle)
{
    uint8_t sector;
    fs_priv_t *fs_priv = fs_priv_handle->fs_priv;

    /* Find a free allocation unit */
    sector = find_free_allocation_unit(fs_priv);
    if ((uint8_t)FS_PRIV_NOT_ALLOCATED == sector)
        return FS_ERROR_FILESYSTEM_FULL;

    /* Update file system allocation table information for this allocation unit */
    fs_priv->alloc_unit_list[sector].file_info.file_id = fs_priv_handle->file_id;
    fs_priv->alloc_unit_list[sector].file_info.next_allocation_unit = (uint8_t)FS_PRIV_NOT_ALLOCATED;
    fs_priv->alloc_unit_list[sector].file_info.file_flags.mode_flags =
            (fs_priv_handle->flags.mode_flags & FS_FILE_CIRCULAR);
    fs_priv->alloc_unit_list[sector].file_info.file_flags.user_flags =
            fs_priv_handle->flags.user_flags;

    /* Check if a root sector is already set for this handle */
    if ((uint8_t)FS_PRIV_NOT_ALLOCATED == fs_priv_handle->root_allocation_unit)
    {
        /* Assign this sector as the handle's root node */
        fs_priv_handle->root_allocation_unit = sector;

        /* Reset file protect bits */
        fs_priv->alloc_unit_list[sector].file_info.file_protect = 0xFF;
    }
    else
    {
        /* Set file protect bits for new sector using root sector file protect bits */
        fs_priv->alloc_unit_list[sector].file_info.file_protect =
                fs_priv->alloc_unit_list[fs_priv_handle->root_allocation_unit].file_info.file_protect;

        /* Chain newly allocated sector onto the end of the current sector */
        fs_priv->alloc_unit_list[fs_priv_handle->curr_allocation_unit].file_info.next_allocation_unit =
                sector;

        /* Write updated file information header contents to flash for the current sector */
        if (syshal_flash_write(fs_priv->device, &fs_priv->alloc_unit_list[fs_priv_handle->curr_allocation_unit],
                FS_PRIV_SECTOR_ADDR(sector),
                sizeof(fs_priv_file_info_t)))
        {
            return FS_ERROR_FLASH_MEDIA;
        }
    }

    /* Reset handle pointers to start of new sector */
    fs_priv_handle->curr_allocation_unit = sector;
    fs_priv_handle->last_data_offset = 0;
    fs_priv_handle->curr_data_offset = 0;
    fs_priv_handle->curr_session_offset = 0;

    /* Write file information header contents to flash for new sector */
    if (syshal_flash_write(fs_priv->device, &fs_priv->alloc_unit_list[sector],
            FS_PRIV_SECTOR_ADDR(sector),
            sizeof(fs_priv_file_info_t)))
        return FS_ERROR_FLASH_MEDIA;

    return FS_NO_ERROR;
}

/*! \brief Ascertain if the allocation unit is full.
 *
 * A sector is considered full if all usable bytes have been
 * written in the sector or the maximum number of write sessions
 * has been filled in the allocation unit.
 *
 * \param fs_priv_handle[in] pointer to private flash handle.
 * \return true if full, false if not full.
 */
static inline bool is_full(fs_priv_handle_t *fs_priv_handle)
{
    return (fs_priv_handle->curr_session_offset == (uint8_t)FS_PRIV_NOT_ALLOCATED ||
            fs_priv_handle->curr_data_offset >= FS_PRIV_USABLE_SIZE);
}

 /*! \brief Write data to a file handle through its cache.
  *
  * The cache fill policy ensures that the cache may never fill more
  * than the number of bytes to the next page boundary i.e., the cache
  * size is at most 1 full page before all data will be emptied from
  * the cache and written into flash.
  *
  * \param fs_priv_handle[in] pointer to private flash handle.
  * \param src[in] pointer to buffer containing bytes to write.
  * \param size[in] number of bytes to write.
  * \param written[out] pointer for storing number of bytes written.
  * \return \ref FS_NO_ERROR on success.
  * \return \ref FS_ERROR_FLASH_MEDIA on a flash write error.
  */
static int write_through_cache(fs_priv_handle_t *fs_priv_handle, const void *src, uint16_t size, uint16_t *written)
{
    uint16_t cached, page_boundary, sz;

    /* Cache operation
     *
     * Rule 1: Write the cache only when there are at least page_boundary bytes in it.
     * Rule 2: Keep caching data until Rule 1 applies.
     *
     * cache = { 0 ... 512 } => number of bytes in the cache
     * page_boundary = { 0 ... 512 } => number of bytes until the next flash page boundary
     * 0 <= (page_boundary - cache) <= 512 => cache may never exceed page_boundary
     *
     * Example:
     *
     * start => page_boundary = 512, cache = 0
     * write(4) => page_boundary = 512, cache = 4
     * flush() => page_boundary = 508, cache = 0
     * write(56) => page_boundary = 508, cache = 56
     * write(3) => page_boundary = 508, cache = 59
     * write(512) => page_boundary = 512, cache = 63
     * flush() => page_boundary = 449, cache = 0
     * write(512) => page_boundary = 512, cache =  63
     * etc
     */
    cached = cached_bytes(fs_priv_handle);
    page_boundary = FS_PRIV_PAGE_SIZE - (fs_priv_handle->last_data_offset & (FS_PRIV_PAGE_SIZE - 1));
    *written = 0;

    assert(cached <= page_boundary);

    /* Append to the cache up to the limit of the next page boundary */
    sz = MIN((page_boundary - cached), size);
    if (sz > 0)
    {
        /* The size is guaranteed to be non-zero */
        memcpy(&fs_priv_handle->page_cache[cached], src, sz);
        src += sz;
        size -= sz;
        cached += sz;
        *written += sz;
        fs_priv_handle->curr_data_offset += sz;
    }

    /* Cache can never exceed page boundary but we should
     * remove data from the cache once there is sufficient
     * data to write to the next page boundary.
     */
    if (cached == page_boundary)
    {
        /* Write through to page boundary */
        uint32_t address = FS_PRIV_SECTOR_ADDR(fs_priv_handle->curr_allocation_unit) +
                FS_PRIV_ALLOC_UNIT_SIZE + fs_priv_handle->last_data_offset;
        if (syshal_flash_write(fs_priv_handle->fs_priv->device,
                fs_priv_handle->page_cache,
                address,
                page_boundary))
            return FS_ERROR_FLASH_MEDIA;

        /* Advance last write position to the next page boundary */
        fs_priv_handle->last_data_offset += page_boundary;
    }

    if (size > 0)
    {
        /* Cache is guaranteed to be empty */
        memcpy(&fs_priv_handle->page_cache, src, size);
        *written += size;
        fs_priv_handle->curr_data_offset += size;
    }

    return FS_NO_ERROR;
}

/* Exported Functions */

/*! \brief Initialize flash file system.
 *
 * Initializes the associated flash memory device and ensures all
 * file handles are marked as free.
 *
 * \param device[in] flash media device number to use for the file system
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA on flash initialization error.
 * \return \ref FS_ERROR_BAD_DEVICE device number out of range.
 */
int fs_init(uint32_t device)
{
    /* This device parameter is used as an index into the file
     * system device list -- we can't accept devices that are
     * out of range.
     */
    if (device >= FS_PRIV_MAX_DEVICES)
        return FS_ERROR_BAD_DEVICE;

    if (syshal_flash_init(device))
        return FS_ERROR_FLASH_MEDIA;

    /* Mark all handles as free */
    for (unsigned int i = 0; i < FS_PRIV_MAX_HANDLES; i++)
        free_handle(&fs_priv_handle_list[i]);

    return FS_NO_ERROR;
}

/*! \brief Terminates flash file system.
 *
 * Terminates the associated flash memory device.
 *
 * \param device[in] flash media device number to terminate
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA on flash terminate error.
 * \return \ref FS_ERROR_BAD_DEVICE device number out of range.
 */
int fs_term(uint32_t device)
{
    /* This device parameter is used as an index into the file
     * system device list -- we can't accept devices that are
     * out of range.
     */
    if (device >= FS_PRIV_MAX_DEVICES)
        return FS_ERROR_BAD_DEVICE;

    if (syshal_flash_term(device))
        return FS_ERROR_FLASH_MEDIA;
    return FS_NO_ERROR;
}

/*! \brief Mount a flash file system.
 *
 * The mount operation scans all sectors on the flash
 * memory and stores alloction unit headers locally to
 * keep track of the overall file system state.
 *
 * \param device[in] flash media device number to mount file system on
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_BAD_DEVICE if the device number is out of range
 */
int fs_mount(uint32_t device, fs_t *fs)
{
    int ret;

    ret = init_fs_priv(device);
    if (!ret)
        *fs = (fs_t)&fs_priv_list[device];

    return ret;
}

/*! \brief Format a flash file system.
 *
 * All flash media sectors shall be erased and locally stored
 * allocation unit headers shall also be reset.  Note that the
 * allocation counters, for wear levelling, are preserved and
 * incremented as per \ref erase_allocation_unit.
 *
 * \param fs[in] file system object to format
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA on flash media error.
 */
int fs_format(fs_t fs)
{
    int ret;
    fs_priv_t *fs_priv = (fs_priv_t *)fs;

    for (uint8_t sector = 0; sector < FS_PRIV_MAX_SECTORS; sector++)
    {
        ret = erase_allocation_unit(fs_priv, sector);
        if (ret) break;
    }

    return ret;
}

/*! \brief Open a file on the file system.
 *
 * Allows new files to be created or an existing file to
 * be opened.  Newly created files are created write only whereas an
 * existing file may be opened as read only or write only subject
 * to the \ref mode parameter.
 *
 * The \ref user_flags field allows application-specific bits to
 * be stored (created file) and retrieved (existing file).  Only
 * the lower nibble is stored.
 *
 * All files are identified by a unique \ref file_id.
 *
 * Upon opening a file, a file handle is allocated to the user
 * which then allows the \ref fs_read or \ref fs_write functions
 * to be used to access the file.  When writing to a file
 * the data contents can be flushed to the flash memory by
 * using the \ref fs_flush function.  When finished using a
 * file then \ref fs_close function should be called.
 *
 * \param fs[in] file system on which to open a file.
 * \param handle[out] pointer for storing allocated file handle.
 * \param file_id[in] file identifier to open.
 * \param mode[in] file open mode.
 * \param user_flags sets the user flags on create, otherwise
 * retrieves the user flags.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FILE_NOT_FOUND if the \ref file_id was not
 * found.
 * \return \ref FS_ERROR_FILE_ALREADY_EXISTS if the \ref file_id has
 * already been created.
 * \return \ref FS_ERROR_FILE_PROTECTED if the \ref file_id has
 * been protected and thus can not be opened in write mode.
 * \return \ref FS_ERROR_NO_FREE_HANDLE if no file handle could be
 * allocated.
 */
int fs_open(fs_t fs, fs_handle_t *handle, uint8_t file_id, fs_mode_t mode, uint8_t *user_flags)
{
    int ret;
    fs_priv_t *fs_priv = (fs_priv_t *)fs;
    fs_priv_handle_t *fs_priv_handle;

    /* Find the root allocation unit for this file (if file exists) */
    uint8_t root = find_file_root(fs_priv, file_id);

    /* Check file identifier versus requested open mode */
    ret = check_file_flags(fs_priv, root, mode);
    if (ret)
        return ret;

    /* Allocate a free handle */
    ret = allocate_handle(fs_priv, &fs_priv_handle);
    if (ret)
        return ret;

    /* Reset file handle */
    *handle = fs_priv_handle;
    fs_priv_handle->file_id = file_id;
    fs_priv_handle->root_allocation_unit = (uint8_t)FS_PRIV_NOT_ALLOCATED;

    if (root != (uint8_t)FS_PRIV_NOT_ALLOCATED)
    {
        /* Existing file: populate file handle */
        fs_priv_handle->root_allocation_unit = root;
        fs_priv_handle->flags.user_flags = get_user_flags(fs_priv, root);
        fs_priv_handle->flags.mode_flags = get_mode_flags(fs_priv, root) | mode;

        /* Retrieve existing user flags if requested */
        if (user_flags)
            *user_flags = fs_priv_handle->flags.user_flags;

        if ((mode & FS_FILE_WRITEABLE) == 0)
        {
            /* Read only - reset to beginning of root sector */
            fs_priv_handle->curr_data_offset = 0;
            fs_priv_handle->curr_allocation_unit = root;

            /* Find the last known write position in this sector so we
             * can check for when to advance to next sector or catch EOF
             */
            find_next_session_offset(fs_priv, root,
                    &fs_priv_handle->last_data_offset);
        }
        else
        {
            /* Write only: find end of file for appending new data */
            fs_priv_handle->curr_session_offset = find_eof(fs_priv, root,
                    &fs_priv_handle->curr_allocation_unit,
                    &fs_priv_handle->curr_data_offset);

            /* Page write cache should be marked empty */
            fs_priv_handle->last_data_offset = fs_priv_handle->curr_data_offset;
        }
    }
    else
    {
        /* Set file flags since we are creating a new file */
        fs_priv_handle->flags.mode_flags = mode;
        fs_priv_handle->flags.user_flags = user_flags ? *user_flags : 0;

        /* Allocate new sector to file handle */
        ret = allocate_new_sector_to_file(fs_priv_handle);
        if (ret)
            free_handle(fs_priv_handle);
    }

    return ret;
}

/*! \brief Close an open file handle.
 *
 * Shall try to execute a flush operation and then frees
 * the file handle.  Note that any error during the
 * flush operation shall not be seen at this level so
 * it is advisable to flush separately first before
 * closing the file handle.
 *
 * \param handle[in] file handle to close.
 * \return \ref FS_NO_ERROR on success.
 */
int fs_close(fs_handle_t handle)
{
    fs_priv_handle_t *fs_priv_handle = (fs_priv_handle_t *)handle;
    fs_flush(handle);
    free_handle(fs_priv_handle);

    return FS_NO_ERROR;
}

/*! \brief Write data to an open file handle.
 *
 * Data bytes shall be written to the file handle via
 * a page cache.  The page cache will write-through to
 * flash memory whenever current write pointer reaches a
 * page boundary and there is data in the cache.
 *
 * The session write pointer is not stored to flash unless the current
 * sector is filled whereupon a new sector is allocated.  Use
 * \ref fs_flush to periodically store the session write pointer
 * to flash.
 *
 * The write process may continue indefinitely if the file
 * type is circular.  In the case of a circular file type,
 * the root sector is reclaimed.  Otherwise, once all
 * sectors have been exhausted a \ref FS_ERROR_FILESYSTEM_FULL
 * error is returned.
 *
 * \param handle[in] file handle to write to.
 * \param src[in] pointer to memory containing bytes to write.
 * \param size[in] number of bytes to write.
 * \param written[out] pointer for storing number of bytes actually
 * written, which may be less than the amount requested.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA on flash write error.
 * \return \ref FS_ERROR_FILESYSTEM_FULL unable to write all data as the
 * file system is full.
 */
int fs_write(fs_handle_t handle, const void *src, uint32_t size, uint32_t *written)
{
    int ret = FS_NO_ERROR;
    fs_priv_handle_t *fs_priv_handle = (fs_priv_handle_t *)handle;
    uint16_t write_size, actual_write;

    /* Reset counter */
    *written = 0;

    /* Check the file is writable */
    if ((fs_priv_handle->flags.mode_flags & FS_FILE_WRITEABLE) == 0)
        return FS_ERROR_INVALID_MODE;

    while (size > 0 && !ret)
    {
        /* Check if the current sector is full */
        if (is_full(fs_priv_handle))
        {
            /* Flush file to clear cache and update session write offset */
            flush_handle(fs_priv_handle);

            /* Allocate new sector to file chain */
            ret = allocate_new_sector_to_file(fs_priv_handle);
            if (ret) return ret;
        }

        /* The permitted write size is limited by the page size and also the
         * number of free bytes remaining in this sector i.e., we don't
         * permit the cache to fill above the sector size since we might not
         * be able to allocate a new sector for the file if no sectors
         * are free i.e., no hidden data loss allowed.
         */
        write_size = MIN(FS_PRIV_PAGE_SIZE, size);
        write_size = MIN(remaining_bytes(fs_priv_handle), write_size);

        /* Write data through the cache.  Note that the actual write size
         * could be less than requested if a flash media error occurred i.e.,
         * we won't try to fill the cache on a flash media error to prevent
         * hidden data loss.
         */
        ret = write_through_cache(fs_priv_handle, src, write_size, &actual_write);
        src += actual_write;
        size -= actual_write;
        *written += actual_write;
    }

    return ret;
}

/*! \brief Read data from an open file handle.
 *
 * The requested number of bytes shall be read from the file.
 * The file's read position shall be updated accordingly.
 * The actual number of bytes read may be less than the number of
 * bytes requested e.g., EOF.
 *
 * \param handle[in] file handle to read.
 * \param dest[out] pointer to buffer for storing read data.
 * \param size[in] number of bytes to read.
 * \param read[out] pointer for storing number of bytes actually read.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA if a flash read failed.
 * \return \ref FS_ERROR_INVALID_MODE if the file handle is not readable.
 * \return \ref FS_ERROR_END_OF_FILE if the end of file has already been reached.
 */
int fs_read(fs_handle_t handle, void *dest, uint32_t size, uint32_t *read)
{
    fs_priv_handle_t *fs_priv_handle = (fs_priv_handle_t *)handle;
    fs_priv_t *fs_priv = fs_priv_handle->fs_priv;

    /* Reset counter */
    *read = 0;

    /* Check the file is read only */
    if (fs_priv_handle->flags.mode_flags & FS_FILE_WRITEABLE)
        return FS_ERROR_INVALID_MODE;

    /* Check for end of file */
    if (is_eof(fs_priv_handle))
        return FS_ERROR_END_OF_FILE;

    while (size > 0)
    {
        /* Check to see if we need to move to the next sector in the file chain */
        if (fs_priv_handle->last_data_offset == fs_priv_handle->curr_data_offset)
        {
            /* Check if we reached the end of the file chain */
            if (is_last_allocation_unit(fs_priv, fs_priv_handle->curr_allocation_unit))
                break;

            /* Not the end of the file chain */
            uint8_t sector = next_allocation_unit(fs_priv, fs_priv_handle->curr_allocation_unit);

            /* Find the last known write position in this sector so we
             * can check for when to advance to next sector or catch EOF
             */
            find_next_session_offset(fs_priv, sector, &fs_priv_handle->last_data_offset);

            /* Reset data offset pointer */
            fs_priv_handle->curr_allocation_unit = sector;
            fs_priv_handle->curr_data_offset = 0;
        }

        /* Read as many bytes as possible from this sector */
        uint32_t read_size = MIN(size, (fs_priv_handle->last_data_offset - fs_priv_handle->curr_data_offset));
        uint32_t address = FS_PRIV_SECTOR_ADDR(fs_priv_handle->curr_allocation_unit) +
                FS_PRIV_FILE_DATA_REL_ADDRESS +
                fs_priv_handle->curr_data_offset;
        if (syshal_flash_read(fs_priv->device,
                dest,
                address,
                read_size))
            return FS_ERROR_FLASH_MEDIA;
        *read += read_size;
        size -= read_size;
        fs_priv_handle->curr_data_offset += read_size;
    }

    return FS_NO_ERROR;
}

/*! \brief Flush an open file.
 *
 * Shall result in any cached data being purged from the
 * page cache and being written to the underlying flash memory.
 * The current session information shall also be stored to
 * flash memory to reflect a new write position, if
 * changed.
 *
 * \param handle[in] file handle to flush.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FLASH_MEDIA if a flash write failed.
 * \return \ref FS_ERROR_INVALID_MODE if the file handle is not writeable.
 * \return \ref FS_ERROR_FILE_PROTECTED if the file is protected.
 */
int fs_flush(fs_handle_t handle)
{
    fs_priv_handle_t *fs_priv_handle = (fs_priv_handle_t *)handle;

    /* Make sure the file is writeable */
    if ((fs_priv_handle->flags.mode_flags & FS_FILE_WRITEABLE) == 0)
        return FS_ERROR_INVALID_MODE;

    /* Flush the handle */
    return flush_handle(fs_priv_handle);
}

/*! \brief Protect a file from the file system.
 *
 * If \ref file_id references a file that can be found on
 * the flash memory and the file is not protected, then
 * the file's protection bits shall be modified.
 *
 * \param fs[in] file system on which to perform protect operation.
 * \param file_id[in] file identifier to protect.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FILE_NOT_FOUND if the \ref file_id was not found.
 * \return \ref FS_ERROR_FLASH_MEDIA if a flash write failed.
 */
int fs_protect(fs_t fs, uint8_t file_id)
{
    fs_priv_t *fs_priv = (fs_priv_t *)fs;

    /* Find the root allocation unit for this file */
    uint8_t root = find_file_root(fs_priv, file_id);
    if ((uint8_t)FS_PRIV_NOT_ALLOCATED == root)
        return FS_ERROR_FILE_NOT_FOUND;

    /* No action needed if already protected */
    if (is_protected(get_file_protect(fs_priv, root)))
        return FS_NO_ERROR;

    uint8_t file_protect = get_file_protect(fs_priv, root);
    file_protect = set_protected(true, file_protect);

    /* Write updated file protect bits to flash */
    if (syshal_flash_write(fs_priv->device, &file_protect,
            FS_PRIV_SECTOR_ADDR(root) + FS_PRIV_FILE_PROTECT_OFFSET,
            sizeof(uint8_t)))
        return FS_ERROR_FLASH_MEDIA;

    fs_priv->alloc_unit_list[root].file_info.file_protect = file_protect;

    return FS_NO_ERROR;
}

/*! \brief Unprotect a file from the file system.
 *
 * If \ref file_id references a file that can be found on
 * the flash memory and the file is protected, then
 * the file's protection bits shall be modified.
 *
 * \param fs[in] file system on which to perform unprotect operation.
 * \param file_id[in] file identifier to unprotect.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FILE_NOT_FOUND if the \ref file_id was not found.
 * \return \ref FS_ERROR_FLASH_MEDIA if a flash write failed.
 */
int fs_unprotect(fs_t fs, uint8_t file_id)
{
    fs_priv_t *fs_priv = (fs_priv_t *)fs;

    /* Find the root allocation unit for this file */
    uint8_t root = find_file_root(fs_priv, file_id);
    if ((uint8_t)FS_PRIV_NOT_ALLOCATED == root)
        return FS_ERROR_FILE_NOT_FOUND;

    /* No action needed if already unprotected */
    if (!is_protected(get_file_protect(fs_priv, root)))
        return FS_NO_ERROR;

    uint8_t file_protect = get_file_protect(fs_priv, root);
    file_protect = set_protected(false, file_protect);

    /* Write updated file protect bits to flash */
    if (syshal_flash_write(fs_priv->device, &file_protect,
            FS_PRIV_SECTOR_ADDR(root) + FS_PRIV_FILE_PROTECT_OFFSET,
            sizeof(uint8_t)))
        return FS_ERROR_FLASH_MEDIA;

    fs_priv->alloc_unit_list[root].file_info.file_protect = file_protect;

    return FS_NO_ERROR;
}

/*! \brief Delete a file from the file system.
 *
 * If \ref file_id references a file that can be found on
 * the flash memory and the file is not protected, then
 * the file shall be removed.  This entails erasing every
 * sector in flash associated with the file.
 *
 * \param fs[in] file system on which to perform file delete.
 * \param file_id[in] file identifier to delete.
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FILE_NOT_FOUND if the \ref file_id was not found.
 * \return \ref FS_ERROR_FILE_PROTECTED if the file is protected.
 */
int fs_delete(fs_t fs, uint8_t file_id)
{
    int ret;
    fs_priv_t *fs_priv = (fs_priv_t *)fs;

    /* Find the root allocation unit for this file */
    uint8_t root = find_file_root(fs_priv, file_id);
    if ((uint8_t)FS_PRIV_NOT_ALLOCATED == root)
        return FS_ERROR_FILE_NOT_FOUND;

    /* Make sure the file is not protected */
    if (is_protected(get_file_protect(fs_priv, root)))
        return FS_ERROR_FILE_PROTECTED;

    /* Erase each allocation unit associated with the file */
    while ((uint8_t)FS_PRIV_NOT_ALLOCATED != root)
    {
        uint8_t temp = root;

        /* Grab next sector in file chain before erasing this one */
        root = next_allocation_unit(fs_priv, root);

        /* This will erase both the flash sector and the local copy
         * of the allocation unit's header.
         */
        ret = erase_allocation_unit(fs_priv, temp);
        if (ret)
            return ret;
    }

    return FS_NO_ERROR;
}

/*! \brief Obtain file or file system status.
 *
 * If \ref file_id is set to \ref FS_FILE_ID_NONE then the operation
 * shall be performed with respect to the file system.  The only
 * field populated shall be \ref fs_stat_t.size and this shall
 * reflect the total number of free bytes in the file system.
 *
 * Otherwise, the specific \ref file_id is retrieved and its
 * total size is scanned along with the file's attributes.
 *
 * \param fs[in] file system on which to perform stat operation.
 * \param file_id[in] shall be FS_FILE_ID_NONE or a valid file identifier.
 * \param stat[out] pointer to file status structure
 * \return \ref FS_NO_ERROR on success.
 * \return \ref FS_ERROR_FILE_NOT_FOUND if the \ref file_id was not
 * found.
 */
int fs_stat(fs_t fs, uint8_t file_id, fs_stat_t *stat)
{
    fs_priv_t *fs_priv = (fs_priv_t *)fs;

    if (FS_FILE_ID_NONE == file_id)
    {
        stat->size = 0; /* Reset size to zero before we start */

        /* Find the total amount of free space on the disk */
        for (uint8_t sector = 0; sector < FS_PRIV_MAX_SECTORS; sector++)
        {
            if (get_file_id(fs_priv, sector) == (uint8_t)FS_PRIV_NOT_ALLOCATED)
            {
                /* Spare sector, so increment by the sector size less the
                 * space we allocate for the file allocation unit management.
                 */
                stat->size += (FS_PRIV_SECTOR_SIZE - FS_PRIV_ALLOC_UNIT_SIZE);
            }
            else
            {
                /* Used sector, so retrieve how much data in the sector has
                 * been used and deduct this from the sector size.
                 */
                uint32_t data_offset;
                if (find_next_session_offset(fs_priv, sector, &data_offset) !=
                        (uint8_t)FS_PRIV_NOT_ALLOCATED)
                    stat->size += ((FS_PRIV_SECTOR_SIZE - FS_PRIV_ALLOC_UNIT_SIZE) -
                            data_offset);
            }
        }
    }
    else
    {
        /* A file identifier was provided, so find its root sector */
        uint8_t root = find_file_root(fs_priv, file_id);
        if ((uint8_t)FS_PRIV_NOT_ALLOCATED == root)
            return FS_ERROR_FILE_NOT_FOUND;

        /* These fields are all in the header of the root sector */
        stat->is_circular = get_mode_flags(fs_priv, root);
        stat->user_flags = get_user_flags(fs_priv, root);
        stat->is_protected = is_protected(get_file_protect(fs_priv, root));

        /* We have to compute the size of the file on the disk iteratively by
         * visiting each sector in the file chain.
         */
        stat->size = 0; /* Reset size to zero before we start */

        while ((uint8_t)FS_PRIV_NOT_ALLOCATED != root)
        {
            uint32_t data_offset;
            find_next_session_offset(fs_priv, root, &data_offset);
            stat->size += data_offset;
            root = next_allocation_unit(fs_priv, root);
        }
    }

    return FS_NO_ERROR;
}
