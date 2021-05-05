/**
 * @file
 * Contains a number of types commonly used by the ZAF.
 *
 * @warning To keep dependencies clean, this header file MUST NOT depend on any other header files
 *          than standard header files or Z-Wave API header files (located in ZWave/API).
 *
 * @copyright 2018 Silicon Laboratories Inc.
 */

#ifndef ZAF_APPLICATIONUTILITIES_ZAF_TYPES_H_
#define ZAF_APPLICATIONUTILITIES_ZAF_TYPES_H_

#include <ZW_security_api.h>
#include <ZW_classcmd.h>

typedef struct _MULTICHAN_SOURCE_NODE_ID_
{
  node_id_t nodeId;         /* The ID of the source node. */
  /* uint8_t 1, bit 0-6 -
   * Can represent endpoints from 1-127 unless used as a bitmask where endpoint 1-7 is represented.*/
  uint8_t   endpoint   : 7;
  /* uint8_t 1, bit 7   - Reserved */
  uint8_t   res        : 1;
} MULTICHAN_SOURCE_NODE_ID;

typedef struct _MULTICHAN_DEST_NODE_ID_
{
  node_id_t nodeId;         /* The ID of the destination node. */
  /* uint8_t 1, bit 0-6 -
   * Can represent endpoints from 1-127 unless used as a bitmask where endpoint 1-7 is represented. */
  uint8_t   endpoint   : 7;
  /* uint8_t 1, bit 7   -
   * Interprets the endpoint fields as a bitmask addressing only the first 7 endpoints in destination. */
  uint8_t   BitAddress : 1;
} MULTICHAN_DEST_NODE_ID;

/* 
 * For the implementation of 16bit NodeId for LR, while keeping backward compatibility, the FRAME-structrues
 * of Z-Wave classic where not copied onto ZAF structures but used with pointers, for this reason, this struct
 * was created to keep those part working.
 * KEEP THE USE OF THIS STRUCT TO BARE MINIMUM!
 */
typedef struct
{
  uint8_t   nodeId;         /* The ID of the destination node. */
  /* uint8_t 1, bit 0-6 -
   * Can represent endpoints from 1-127 unless used as a bitmask where endpoint 1-7 is represented. */
  uint8_t   endpoint   : 7;
  /* uint8_t 1, bit 7   -
   * Interprets the endpoint fields as a bitmask addressing only the first 7 endpoints in destination. */
  uint8_t   BitAddress : 1;
} MULTICHAN_DEST_NODE_ID_8bit;

/**
 * Properties of the received frame. Used mostly by command classes to prepare response to a command.
 */
typedef struct _RECEIVE_OPTIONS_TYPE_EX_
{
  uint8_t rxStatus;                       ///< Frame header info
  security_key_t securityKey;
  MULTICHAN_SOURCE_NODE_ID sourceNode;
  MULTICHAN_DEST_NODE_ID destNode;
  uint8_t bSupervisionActive       : 1;   ///< true if supervision is active
  uint8_t sessionId                : 6;   ///< Current sessionId
  uint8_t statusUpdate             : 1;   ///< Is statusUpdate enabled for current session (see cc_supervision_status_updates_t)
} RECEIVE_OPTIONS_TYPE_EX;


/**
 * @enum e_cmd_handler_return_code_t
 */
typedef enum
{
  E_CMD_HANDLER_RETURN_CODE_FAIL,           ///< the command was not accepted or accepted but failed to execute. Command class returns FAIL
  E_CMD_HANDLER_RETURN_CODE_HANDLED,        ///< the command was accepted and executed by the command handler. Command class returns SUCCESS
  E_CMD_HANDLER_RETURN_CODE_WORKING,        ///< the command was accepted but is not fully executed. Command class returns WORKING
  E_CMD_HANDLER_RETURN_CODE_NOT_SUPPORTED,  ///< the command handler does not support this command. Command class returns NO_SUPPORT
} e_cmd_handler_return_code_t;

/**
 * Status on incoming frame. Use same values as cc_supervision_status_t
 */
typedef enum
{
  RECEIVED_FRAME_STATUS_NO_SUPPORT = 0x00, ///< Frame not supported
  RECEIVED_FRAME_STATUS_WORKING = 0x01,    ///< Frame received successfully and timed change started
  RECEIVED_FRAME_STATUS_FAIL = 0x02,       ///< Could not handle incoming frame
  RECEIVED_FRAME_STATUS_CANCEL = 0x03,     ///< Don't care about status. CC or App will send the report
  RECEIVED_FRAME_STATUS_SUCCESS = 0xFF     ///< Frame received successfully
} received_frame_status_t;

/**
 * Can be used for pairing a command class with a specific command in the command class.
 */
typedef struct
{
  uint8_t cmdClass; /**< Command class */
  uint8_t cmd;      /**< Command */
} cc_group_t;

/**
 * For backwards compatibility.
 */
typedef cc_group_t CMD_CLASS_GRP;

/**
 * Can be used for pairing AGI profile identifiers listed in \cite SDS12657.
 */
typedef struct
{
  uint8_t profile_MS; /**< AGI profile of type: ASSOCIATION_GROUP_INFO_REPORT_PROFILE_...*/
  uint8_t profile_LS; /**< AGI profile of type: ASSOCIATION_GROUP_INFO_REPORT_PROFILE_...*/
} agi_profile_t;

/**
 * For backwards compatibility.
 */
typedef agi_profile_t AGI_PROFILE;

/**
 * Callback status used on framework API for request/response-job
 */
typedef enum
{
  JOB_STATUS_SUCCESS = 0,     /**< Job has been started. */
  JOB_STATUS_BUSY,            /**< Job couldn't start.  */
  JOB_STATUS_NO_DESTINATIONS  /**< Job couldn't start because there is no destinations. */
} job_status_t;

/**
 * For backwards compatibility.
 */
typedef job_status_t JOB_STATUS;

/**
 * Indicates whether all transmissions are done. Used by \ref TRANSMISSION_RESULT.
 */
typedef enum
{
  TRANSMISSION_RESULT_NOT_FINISHED,   /**< Still transmitting. */
  TRANSMISSION_RESULT_FINISHED        /**< Done transmitting to all nodes. */
} TRANSMISSION_RESULT_FINISH_STATUS;

/**
 * This struct defines the values which can be parsed to a callback function
 * upon an ended transmission regardless of the result.
 */
typedef struct
{
  node_id_t nodeId;  /**< The ID of the node to which the transmission has been done. */
  uint8_t   status;  /**< Status of the transmission. See ZW_transport_api.h. */
  /**
   * If transmission to several nodes, this flag is set if transmission for the last node has ended.
   */
  TRANSMISSION_RESULT_FINISH_STATUS isFinished;
} transmission_result_t;

/**
 * For backwards compatibility.
 */
typedef transmission_result_t TRANSMISSION_RESULT;

typedef struct
{
  uint16_t CC;
  uint16_t version;
  received_frame_status_t(*pHandler)(RECEIVE_OPTIONS_TYPE_EX *, ZW_APPLICATION_TX_BUFFER *, uint8_t);
}
CC_handler_map_t;


#ifdef __APPLE__
#define HANDLER_SECTION "__TEXT,__cc_handlers"
extern CC_handler_map_t __start__cc_handlers __asm("section$start$__TEXT$__cc_handlers");
extern CC_handler_map_t __stop__cc_handlers __asm("section$end$__TEXT$__cc_handlers");
#else
#define HANDLER_SECTION "_cc_handlers"
/**
 * This is the first of the registered app handlers
 */
extern const CC_handler_map_t __start__cc_handlers;
/**
 * This marks the end of the handlers. The element
 * after the last element. This means that this element
 * is not valid.
 */
extern const CC_handler_map_t __stop__cc_handlers;
#endif


/**
 * Every CC must register itself using the REGISTER_CC macro.
 *
 * This macro will locate a variable containing
 * - the CC number,
 * - the CC version, and
 * - the CC handler
 * in a specific code section.
 * This section (array) can then be looped through to access the handler, the version and other
 * stuff that might be added in the future.
 */
#define REGISTER_CC(cc,version,handler) \
  static const CC_handler_map_t thisHandler##cc __attribute__((__used__, __section__( HANDLER_SECTION ))) = {cc,version,handler};

#endif /* ZAF_APPLICATIONUTILITIES_ZAF_TYPES_H_ */
