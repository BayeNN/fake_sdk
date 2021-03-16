/***************************************************************************//**
 * @file app_psa_crypto_ecdsa.h
 * @brief PSA Crypto ECDSA functions.
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
#ifndef APP_PSA_CRYPTO_ECDSA_H
#define APP_PSA_CRYPTO_ECDSA_H

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include "app_psa_crypto_key.h"
#include "app_psa_crypto_macro.h"
#if defined(MBEDTLS_PSA_CRYPTO_BUILTIN_KEYS)
#include "sli_se_opaque_types.h"
#endif

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
/// ECDSA signature size
#define SIGNATURE_SIZE          (132)

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Public Function Declarations
// -----------------------------------------------------------------------------

/***************************************************************************//**
 * Set pointer to message or hash buffer.
 *
 * @param ptr The pointer to message or hash buffer.
 ******************************************************************************/
void set_msg_hash_buf_ptr(uint8_t *ptr);

/***************************************************************************//**
 * Set message or hash length.
 *
 * @param length The length of the message or hash.
 ******************************************************************************/
void set_msg_hash_len(size_t length);

/***************************************************************************//**
 * Set ECDSA algorithm.
 *
 * @param alg The ECDSA algorithm to be used.
 ******************************************************************************/
void set_ecdsa_algo(psa_algorithm_t alg);

/***************************************************************************//**
 * Sign an already-calculated hash or short message with a private key.
 *
 * @returns Returns PSA error code, @ref crypto_values.h.
 ******************************************************************************/
psa_status_t sign_hash(void);

/***************************************************************************//**
 * Verify the signature of a hash or short message using a public key.
 *
 * @returns Returns PSA error code, @ref crypto_values.h.
 ******************************************************************************/
psa_status_t verify_hash(void);

#endif  // APP_PSA_CRYPTO_ECDSA_H
