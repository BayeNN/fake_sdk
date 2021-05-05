/***************************************************************************//**
 * @file sli_wisun_nvm.c
 * @brief Wi-SUN NVM API
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <string.h>
#include "sli_wisun_trace.h"
#include "nvm3.h"
#include "ns_list.h"
#include "nsdynmemLIB.h"
#include "Security/protocols/sec_prot_certs.h"
#include "Security/protocols/sec_prot_keys.h"
#include "6LoWPAN/ws/ws_pae_nvm_store.h"
#include "6LoWPAN/ws/ws_pae_nvm_data.h"
#include "common_functions.h"
#include "sli_wisun_nvm.h"

#define TRACE_GROUP "WISUNNVM"

#define WLI_WISUN_NVM_KEY_BASE     0x00090000
#define WLI_WISUN_NVM_KEY_NW_INFO  0x00090010
#define WLI_WISUN_NVM_KEY_KEYS     0x00090020
#define WLI_WISUN_NVM_KEY_COUNTERS 0x00090030

static int8_t sli_wisun_nvm_read(nvm3_ObjectKey_t tlv_key, nvm_tlv_t *tlv)
{
  uint32_t tlv_type;
  size_t tlv_size;
  Ecode_t ret;

  // NW info is stored in a single blob
  ret = nvm3_getObjectInfo(nvm3_defaultHandle,
                           tlv_key,
                           &tlv_type,
                           &tlv_size);
  if (ret != ECODE_NVM3_OK) {
    tr_error("Unable to find key 0x%08lx: 0x%08lx", tlv_key, ret);
    goto failure;
  }

  if (tlv_size != tlv->len + NVM_TLV_FIXED_LEN) {
    tr_error("Stored length: %u, requested length: %u", tlv_size, tlv->len + NVM_TLV_FIXED_LEN);
    goto failure;
  }

  ret = nvm3_readData(nvm3_defaultHandle,
                      tlv_key,
                      &tlv->tag,
                      tlv_size);
  if (ret != ECODE_NVM3_OK) {
    tr_error("Unable to read key 0x%08lx: 0x%08lx", tlv_key, ret);
    goto failure;
  }

  tr_debug("sli_wisun_nvm_read[0x%08lx]: %u", tlv_key, tlv_size);
  return 0;

failure:

  return -1;
}

static int8_t sli_wisun_nvm_write(nvm3_ObjectKey_t tlv_key, nvm_tlv_t *tlv)
{
  Ecode_t ret;

  if (!tlv) {
    tr_error("TLV list is empty");
    goto failure;
  }

  ret = nvm3_writeData(nvm3_defaultHandle,
                       tlv_key,
                       &tlv->tag,
                       tlv->len + NVM_TLV_FIXED_LEN);
  if (ret != ECODE_NVM3_OK) {
    tr_error("Unable to write key 0x%08lx: 0x%08lx", tlv_key, ret);
    goto failure;
  }

  tr_debug("sli_wisun_nvm_write[0x%08lx]: %u", tlv_key, tlv->len + NVM_TLV_FIXED_LEN);
  return 0;

failure:

  return -1;
}

static int8_t sli_wisun_nvm_delete(nvm3_ObjectKey_t tlv_key)
{
  Ecode_t ret;

  ret = nvm3_deleteObject(nvm3_defaultHandle,
                          tlv_key);
  if (ret != ECODE_NVM3_OK) {
    tr_error("Unable to delete key 0x%08lx: 0x%08lx", tlv_key, ret);
    goto failure;
  }

  tr_debug("sli_wisun_nvm_delete[0x%08lx]", tlv_key);
  return 0;

failure:

  return -1;
}

void sli_wisun_nvm_init()
{
}

void sli_wisun_nvm_clear()
{
  (void)sli_wisun_nvm_delete(WLI_WISUN_NVM_KEY_NW_INFO);
  (void)sli_wisun_nvm_delete(WLI_WISUN_NVM_KEY_KEYS);
  (void)sli_wisun_nvm_delete(WLI_WISUN_NVM_KEY_COUNTERS);
}

int8_t ws_pae_nvm_store_tlv_file_write(const char *file, nvm_tlv_t *tlv)
{
  tr_debug("ws_pae_nvm_store_tlv_file_write: %s", file);

  if (!strcmp(file, NW_INFO_FILE_NAME)) {
    return sli_wisun_nvm_write(WLI_WISUN_NVM_KEY_NW_INFO, tlv);
  } else if (!strcmp(file, KEYS_FILE_NAME)) {
    return sli_wisun_nvm_write(WLI_WISUN_NVM_KEY_KEYS, tlv);
  } else if (!strcmp(file, FRAME_COUNTER_FILE_NAME)) {
    return sli_wisun_nvm_write(WLI_WISUN_NVM_KEY_COUNTERS, tlv);
  }

  return -1;
}

int8_t ws_pae_nvm_store_tlv_file_read(const char *file, nvm_tlv_t *tlv)
{
  tr_debug("ws_pae_nvm_store_tlv_file_read: %s", file);

  if (!strcmp(file, NW_INFO_FILE_NAME)) {
    return sli_wisun_nvm_read(WLI_WISUN_NVM_KEY_NW_INFO, tlv);
  } else if (!strcmp(file, KEYS_FILE_NAME)) {
    return sli_wisun_nvm_read(WLI_WISUN_NVM_KEY_KEYS, tlv);
  } else if (!strcmp(file, FRAME_COUNTER_FILE_NAME)) {
    return sli_wisun_nvm_read(WLI_WISUN_NVM_KEY_COUNTERS, tlv);
  }

  return -1;
}

int8_t ws_pae_nvm_store_tlv_file_remove(const char *file)
{
  tr_debug("ws_pae_nvm_store_tlv_file_remove: %s", file);

  if (!strcmp(file, NW_INFO_FILE_NAME)) {
    return sli_wisun_nvm_delete(WLI_WISUN_NVM_KEY_NW_INFO);
  } else if (!strcmp(file, KEYS_FILE_NAME)) {
    return sli_wisun_nvm_delete(WLI_WISUN_NVM_KEY_KEYS);
  } else if (!strcmp(file, FRAME_COUNTER_FILE_NAME)) {
    return sli_wisun_nvm_delete(WLI_WISUN_NVM_KEY_COUNTERS);
  }

  return -1;
}

