 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

#include "Common_Macros.h"
#include "Std_Types.h"
#include "Port_Cfg.h"

/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/
   
#define PORT_VENDOR_ID_H  (2000U)

/* Port Module Id */
#define PORT_MODULE_ID    (124U)

/* PORT Instance Id */
#define PORT_INSTANCE_ID  (0U)
/*
 * SW Module Version is 1.0.0
 */
#define PORT_SW_MAJOR_VERSION             (1U)
#define PORT_SW_MINOR_VERSION             (0U)
#define PORT_SW_PATCH_VERSION             (0U)

/*
 * AUTOSAR Version is 4.0.3 compatible
 */
#define PORT_AR_RELEASE_MAJOR_VERSION    (4U)
#define PORT_AR_RELEASE_MINOR_VERSION    (0U)
#define PORT_AR_RELEASE_PATCH_VERSION    (3U)


/* AUTOSAR checking between Std Types and PORT Modules */

#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* AUTOSAR Version checking between PORT_Cfg.h and PORT.h files */

#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

   /* Software Version checking between PORT_Cfg.h and PORT.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif   
   
   
   
/*******************************************************************************************
                                 API Service Id 
*********************************************************************************************/

/* Service IDs */

  /* API service ID for PORT Init function.
   */
#define PORT_INIT_ID                    (uint8)0x00

/*   API service ID for PORT set pin direction function.
 */
#define PORT_SETPINDIRECTION_ID         (uint8)0x01

/*  API service ID for PORT refresh pin direction function.
 */
#define PORT_REFRESHPINDIRECTION_ID     (uint8)0x02

/*
 API service ID for PORT get version info function.
 */
#define PORT_GETVERSIONINFO_ID          (uint8)0x03


#define PORT_SETPINMODE_ID              (uint8)0x04   
/***********************************************************************/
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/***************************************************************
 *                       DET Error Codes                       * 
 ***************************************************************/
/* Invalid Port ID request */
#define PORT_E_PARAM_PIN                 ((uint8)0x0A)

/* Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE    ((uint8)0x0B)

/* API Port_Init service called with wrong parameters */
#define PORT_E_PARAM_CONFIG              ((uint8)0x0C)

/* API Port_SetPinMode service called with invalid mode */
#define PORT_E_PARAM_INVALID_MODE        ((uint8)0x0D)

/* API Port_SetPinMode service called, when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE         ((uint8)0x0E)

/* API sevice called without module initialization */
#define PORT_E_UNINIT                    ((uint8)0x0F)

/* APIs called with a NULL Pointer, Error shall be reported */
#define PORT_E_PARAM_POINTER             ((uint8)0x10)

   


/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

typedef uint8 Port_PinType;

typedef uint8 Port_PinModeType;                  // 8 Modes 
   
/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirection;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 *      5. Initial value for the pin
 */

typedef struct 
{
    uint8 port_num; 
    uint8 pin_num; 
    Port_PinDirection direction;
    /*pin direction changeable during runtime*/
    uint8 Pin_direction_Change;
    Port_InternalResistor resistor;
    uint8 initial_value;
    
    /*Pin mode*/
   uint8 pin_mode;
}Port_ConfigPins;

typedef struct
{
	Port_ConfigPins Pins[PORT_NUMBER_OF_PORT_PINS ];      //43 Structure 
}Port_ConfigType;






/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initializes the Port Driver module.
*              - Setup the pins as Digital GPIO pins
*              - Setup the direction of the GPIO pins
*              - Setup the internal resistor for i/p pins
*              - Setup the inintial value for pins  
************************************************************************************/
void Port_Init(const Port_ConfigType *ConfigPtr );

/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in):  Pin > Port Pin ID number 
                    Direction> Port Pin Direction 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Sets the port pin direction.
************************************************************************************/


#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirection Direction );
#endif


#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );
#endif


#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );
#endif

void Port_RefreshPortDirection( void );

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/
extern const Port_ConfigType Port_Configuration;

#endif /* PORT_H */
