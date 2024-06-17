/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.  freeemodbus��:һ������ֲ��Modbusʵ��Modbus ASCII/RTU��
 * Copyright (c) 2006-2018 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* ���ͷ�ļ����������ĺ��� ��
eMBErrorCode    eMBInit( eMBMode eMode, UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity );
eMBErrorCode    eMBTCPInit( USHORT usTCPPort );

eMBErrorCode    eMBClose( void );

eMBErrorCode    eMBEnable( void );
eMBErrorCode    eMBDisable( void );

eMBErrorCode    eMBPoll( void );

eMBErrorCode    eMBSetSlaveID( UCHAR ucSlaveID, BOOL xIsRunning, UCHAR const *pucAdditional, USHORT usAdditionalLen );

eMBErrorCode    eMBRegisterCB( UCHAR ucFunctionCode,  pxMBFunctionHandler pxHandler );
eMBErrorCode    eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs );
eMBErrorCode    eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode );
eMBErrorCode    eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode );
eMBErrorCode    eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete );
*/


#ifndef _MB_H
#define _MB_H

#include "port.h"

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

#include "mbport.h"
#include "mbproto.h"

/*! \defgroup modbus Modbus
 * \code #include "mb.h" \endcode
 *
 * This module defines the interface for the application. 
 * ���ģ�鶨����Ӧ�ó���Ľӿڡ�
 * It contains the basic functions and types required to use the Modbus protocol stack.
 * ������ʹ��ModbusЭ��ջ����Ļ������������͡�
 * A typical application will want to call eMBInit() first.
 * һ�����͵�Ӧ�ó������Ҫ�ȵ���eMBInit()��
 * If the device is ready to answer network requests it must then call eMBEnable() to activate the protocol stack. 
 * ����豸׼������Ӧ����������ô���������eMBEnableO������Э��ջ��
 * In the main loop the function eMBPoll() must be called periodically. 
 * ����ѭ���У����붨�ڵ���eMBPoll()������
 * The time interval between pooling depends on the configured Modbus timeout.
 * �ػ���ʱ����ȡ�������õ�Modbus��ʱʱ�䡣
 * If an RTOS is available a separate task should be created and the task should always call the function eMBPoll().
 * ���RTOS���ã���Ӧ�ô���һ�����������񣬲��Ҹ�����Ӧ��ʼ�յ���eMBPoll()������
 * \code
 * // Initialize protocol stack in RTU mode for a slave with address 10 = 0x0A  Ϊ��ַΪ10 = OxOA�Ĵӻ���ʼ��RTUģʽ��Э��ջ
 * eMBInit( MB_RTU, 0x0A, 38400, MB_PAR_EVEN );
 * // Enable the Modbus Protocol Stack.
 * eMBEnable(  );
 * for( ;; )
 * {
 *     // Call the main polling loop of the Modbus protocol stack. ����ModbusЭ��ջ������ѯѭ����
 *     eMBPoll(  );
 *     ...
 * }
 * \endcode
 */

/* ----------------------- Defines ------------------------------------------*/

/*! \ingroup modbus
 * \brief Use the default Modbus TCP port (502)
 */
#define MB_TCP_PORT_USE_DEFAULT 0   

/* ----------------------- Type definitions ---------------------------------*/

/*! \ingroup modbus
 * \brief Modbus serial transmission modes (RTU/ASCII).  Modbus���д��䷽ʽ(RTU/ASCII)��
 *
 * Modbus serial supports two transmission modes. Either ASCII or RTU.
 * Modbus����֧�����ִ��䷽ʽ��ASCIl��RTU��
 * RTU is faster but has more hardware requirements and requires a network with a low jitter.
 * RTU�ٶȸ��죬����Ӳ����Ҫ����ߣ�������Ҫһ���Ͷ��������硣
 * ASCII is slower and more reliable on slower links (E.g. modems)
 * ASCIl�ڽ�������·(������ƽ����)�ϸ��������ɿ��� 
 */
    typedef enum
{
    MB_RTU,                     /*!< RTU transmission mode. */
    MB_ASCII,                   /*!< ASCII transmission mode. */
    MB_TCP                      /*!< TCP mode. */
} eMBMode;

/*! \ingroup modbus
 * \brief If register should be written or read.    ����Ĵ�����Ҫд����ȡ��
 *
 * This value is passed to the callback functions which support either reading or writing register values.
 * ���ֵ�����ݸ�֧�ֶ���д�Ĵ���ֵ�Ļص�����
 * Writing means that the application registers should be updated 
 * and reading means that the modbus protocol stack needs to know the current register values.
 * д��ζ��Ӧ�ó���ļĴ���Ӧ�ñ�����,��������ζ��modbusЭ��ջ��Ҫ֪����ǰ�ļĴ���ֵ��
 * 
 * \see eMBRegHoldingCB( ), eMBRegCoilsCB( ), eMBRegDiscreteCB( ) and 
 *   eMBRegInputCB( ).
 */
typedef enum
{
    MB_REG_READ,                /*!< Read register values and pass to protocol stack. */
    MB_REG_WRITE                /*!< Update register values. */
} eMBRegisterMode;

/*! \ingroup modbus
 * \brief Errorcodes used by all function in the protocol stack.    Э��ջ�����к���ʹ�õĴ����롣
 */
typedef enum
{
    MB_ENOERR,                  /*!< no error. */
    MB_ENOREG,                  /*!< illegal register address. */
    MB_EINVAL,                  /*!< illegal argument. */
    MB_EPORTERR,                /*!< porting layer error. */
    MB_ENORES,                  /*!< insufficient resources. */
    MB_EIO,                     /*!< I/O error. */
    MB_EILLSTATE,               /*!< protocol stack in illegal state. */
    MB_ETIMEDOUT                /*!< timeout error occurred. */
} eMBErrorCode;


/* ----------------------- Function prototypes ------------------------------*/
/*! \ingroup modbus
 * \brief Initialize the Modbus protocol stack.
 *
 * This functions initializes the ASCII or RTU module and calls the init functions of the porting layer to prepare the hardware. 
 * �ú�����ʼ��ASCII��RTUģ�飬��������ֲ���init������׼��Ӳ����
 * Please note that the receiver is still disabled and no Modbus frames are processed until eMBEnable( ) has been called.
 * ��ע�⣬��������Ȼ���ڽ���״̬���ڵ���eMBEnable()֮ǰ���ᴦ���κ�Modbus֡��
 *
 * \param eMode
 *    If ASCII or RTU mode should be used.    ���ʹ��ASCII��RTUģʽ��
 * \param ucSlaveAddress
 *    The slave address. Only frames sent to this address or to the broadcast address are processed.
 *    �ӻ���ַ��ֻ�з��͵��õ�ַ��㲥��ַ��֡�Żᱻ����
 * \param ucPort 
 *    The port to use. E.g. 1 for COM1 on windows. 
 *    Ҫʹ�õĶ˿ڡ����磬1����windows�ϵ�COM1��
 *    This value is platform dependent and some ports simply choose to ignore it.
 *    ���ֵ��ƽ̨��صģ�һЩ�˿ڸɴ�ѡ���������
 * \param ulBaudRate 
 *    The baudrate. E.g. 19200. Supported baudrates depend on the porting layer.
 *    �����ʡ�����19200��֧�ֵĲ�����ȡ������ֲ�㡣
 * \param eParity 
 *    Parity  used for serial transmission.
 *    ���ڴ��д����У���ԡ�
 *
 * \return 
 *   If no error occurs the function returns eMBErrorCode::MB_ENOERR.
 *   The protocol is then in the disabled state and ready for activation by calling eMBEnable( ). 
 *   Ȼ��Э�鴦�ڽ���״̬����׼��ͨ������eMBEnable()�����
 *   Otherwise one of the following error codes is returned:
 *    - eMBErrorCode::MB_EINVAL 
 *        If the slave address was not valid. Valid slave addresses are in the range 1 - 247.
 *        ����ӵ�ַ��Ч����Ч�Ĵ�����ַ��1 - 247��Χ�ڡ�
 *    - eMBErrorCode::MB_EPORTERR 
 *        IF the porting layer returned an error.
 *        �����ֲ�㷵��һ������
 */
eMBErrorCode    eMBInit( eMBMode eMode, UCHAR ucSlaveAddress,
                         UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity );

/*! \ingroup modbus
 * \brief Initialize the Modbus protocol stack for Modbus TCP.    ΪModbus TCP��ʼ��ModbusЭ��ջ��
 *
 * This function initializes the Modbus TCP Module.
 * ��ʼ��Modbus TCPģ�顣
 * Please note that frame processing is still disabled until eMBEnable( ) is called.
 * ��ע�⣬�ڵ���eMBEnable()֮ǰ��֡������Ȼ�ǽ��õġ�
 *
 * \param usTCPPort The TCP port to listen on.    Ҫ������TCP�˿ڡ�
 * \return 
 *   If the protocol stack has been initialized correctly the function returns eMBErrorCode::MB_ENOERR. 
 *    ���Э��ջ�Ѿ���ȷ��ʼ������������eMBErrorCode::MB_ENOERR��
 *   Otherwise one of the following error codes is returned:    ���򷵻����´�����֮һ:
 *    - eMBErrorCode::MB_EINVAL 
 *        If the slave address was not valid. Valid slave addresses are in the range 1 - 247.
 *    - eMBErrorCode::MB_EPORTERR 
 *        IF the porting layer returned an error.
 */
eMBErrorCode    eMBTCPInit( USHORT usTCPPort );

/*! \ingroup modbus
 * \brief Release resources used by the protocol stack.
 *
 * This function disables the Modbus protocol stack and release all
 * hardware resources. It must only be called when the protocol stack 
 * is disabled. 
 *
 * \note Note all ports implement this function. A port which wants to 
 *   get an callback must define the macro MB_PORT_HAS_CLOSE to 1.
 *
 * \return If the resources where released it return eMBErrorCode::MB_ENOERR.
 *   If the protocol stack is not in the disabled state it returns
 *   eMBErrorCode::MB_EILLSTATE.
 */
eMBErrorCode    eMBClose( void );

/*! \ingroup modbus
 * \brief Enable the Modbus protocol stack.
 *
 * This function enables processing of Modbus frames. Enabling the protocol
 * stack is only possible if it is in the disabled state.
 *
 * \return If the protocol stack is now in the state enabled it returns 
 *   eMBErrorCode::MB_ENOERR. If it was not in the disabled state it 
 *   return eMBErrorCode::MB_EILLSTATE.
 */
eMBErrorCode    eMBEnable( void );

/*! \ingroup modbus
 * \brief Disable the Modbus protocol stack.
 *
 * This function disables processing of Modbus frames.
 *
 * \return If the protocol stack has been disabled it returns 
 *  eMBErrorCode::MB_ENOERR. If it was not in the enabled state it returns
 *  eMBErrorCode::MB_EILLSTATE.
 */
eMBErrorCode    eMBDisable( void );

/*! \ingroup modbus
 * \brief 
 *     The main pooling loop of the Modbus protocol stack.
 *     ModbusЭ��ջ������ѭ����
 *     This function must be called periodically.
 *     ����������붨�ڵ��á�
 *     The timer interval required is given by the application dependent Modbus slave timeout. 
 *     ����Ķ�ʱ�������������Ӧ�ó����Modbus�ӻ���ʱʱ�������
 *     Internally the function calls xMBPortEventGet() and waits for an event from the receiver or transmitter state machines. 
 *     ���ڲ����ú�������xMBPortEventGet()���ȴ����Խ��շ����ͷ�״̬�����¼���
 *
 * \return 
 *     If the protocol stack is not in the enabled state the function returns eMBErrorCode::MB_EILLSTATE.
 *     ���Э��ջδ��������״̬���򷵻أ�MB_EILLSTATE��
 *     Otherwise it returns eMBErrorCode::MB_ENOERR.
 *     ���򷵻أ�MB_ENOERR
 */
eMBErrorCode    eMBPoll( void );

/*! \ingroup modbus
 * \brief Configure the slave id of the device.
 *
 * This function should be called when the Modbus function <em>Report Slave ID</em>
 * is enabled ( By defining MB_FUNC_OTHER_REP_SLAVEID_ENABLED in mbconfig.h ).
 *
 * \param ucSlaveID Values is returned in the <em>Slave ID</em> byte of the
 *   <em>Report Slave ID</em> response.
 * \param xIsRunning If TRUE the <em>Run Indicator Status</em> byte is set to 0xFF.
 *   otherwise the <em>Run Indicator Status</em> is 0x00.
 * \param pucAdditional Values which should be returned in the <em>Additional</em>
 *   bytes of the <em> Report Slave ID</em> response.
 * \param usAdditionalLen Length of the buffer <code>pucAdditonal</code>.
 *
 * \return If the static buffer defined by MB_FUNC_OTHER_REP_SLAVEID_BUF in
 *   mbconfig.h is to small it returns eMBErrorCode::MB_ENORES. Otherwise
 *   it returns eMBErrorCode::MB_ENOERR.
 */
eMBErrorCode    eMBSetSlaveID( UCHAR ucSlaveID, BOOL xIsRunning,
                               UCHAR const *pucAdditional,
                               USHORT usAdditionalLen );

/*! \ingroup modbus
 * \brief Registers a callback handler for a given function code.
 *
 * This function registers a new callback handler for a given function code.
 * The callback handler supplied is responsible for interpreting the Modbus PDU and
 * the creation of an appropriate response. In case of an error it should return
 * one of the possible Modbus exceptions which results in a Modbus exception frame
 * sent by the protocol stack. 
 *
 * \param ucFunctionCode The Modbus function code for which this handler should
 *   be registers. Valid function codes are in the range 1 to 127.
 * \param pxHandler The function handler which should be called in case
 *   such a frame is received. If \c NULL a previously registered function handler
 *   for this function code is removed.
 *
 * \return eMBErrorCode::MB_ENOERR if the handler has been installed. If no
 *   more resources are available it returns eMBErrorCode::MB_ENORES. In this
 *   case the values in mbconfig.h should be adjusted. If the argument was not
 *   valid it returns eMBErrorCode::MB_EINVAL.
 */
eMBErrorCode    eMBRegisterCB( UCHAR ucFunctionCode, 
                               pxMBFunctionHandler pxHandler );

/* ----------------------- Callback -----------------------------------------*/

/*! \defgroup modbus_registers Modbus Registers
 * \code #include "mb.h" \endcode
 * The protocol stack does not internally allocate any memory for the
 * registers. This makes the protocol stack very small and also usable on
 * low end targets. In addition the values don't have to be in the memory
 * and could for example be stored in a flash.<br>
 * Whenever the protocol stack requires a value it calls one of the callback
 * function with the register address and the number of registers to read
 * as an argument. The application should then read the actual register values
 * (for example the ADC voltage) and should store the result in the supplied
 * buffer.<br>
 * If the protocol stack wants to update a register value because a write
 * register function was received a buffer with the new register values is
 * passed to the callback function. The function should then use these values
 * to update the application register values.
 */

/*! \ingroup modbus_registers
 * \brief Callback function used if the value of a <em>Input Register</em>
 *   is required by the protocol stack. The starting register address is given
 *   by \c usAddress and the last register is given by <tt>usAddress +
 *   usNRegs - 1</tt>.
 *
 * \param pucRegBuffer A buffer where the callback function should write
 *   the current value of the modbus registers to.
 * \param usAddress The starting address of the register. Input registers
 *   are in the range 1 - 65535.
 * \param usNRegs Number of registers the callback function must supply.
 *
 * \return The function must return one of the following error codes:
 *   - eMBErrorCode::MB_ENOERR If no error occurred. In this case a normal
 *       Modbus response is sent.
 *   - eMBErrorCode::MB_ENOREG If the application can not supply values
 *       for registers within this range. In this case a 
 *       <b>ILLEGAL DATA ADDRESS</b> exception frame is sent as a response.
 *   - eMBErrorCode::MB_ETIMEDOUT If the requested register block is
 *       currently not available and the application dependent response
 *       timeout would be violated. In this case a <b>SLAVE DEVICE BUSY</b>
 *       exception is sent as a response.
 *   - eMBErrorCode::MB_EIO If an unrecoverable error occurred. In this case
 *       a <b>SLAVE DEVICE FAILURE</b> exception is sent as a response.
 */
eMBErrorCode    eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress,
                               USHORT usNRegs );

/*! \ingroup modbus_registers
 * \brief 
 *   Callback function used if a <em>Holding Register</em> value is read or written by the protocol stack. 
 *   ��<em>���ּĴ���</em>ֵ��Э��ջ��ȡ��д��ʱʹ�õĻص�����
 *   The starting register address is given by \c usAddress and the last register is given by <tt>usAddress + usNRegs - 1</tt>.
 *   ��ʼ�Ĵ�����ַ��\c usAddress������ĩβ�Ĵ�����<tt>usAddress + usNRegs - 1</tt>������
 *
 * \param pucRegBuffer
 *			If the application registers values should be updated the buffer points to the new registers values.             д
 *          ���Ӧ�ó���ļĴ���ֵ��Ҫ���£��򻺳���ָ���µļĴ���ֵ��
 *			If the protocol stack needs to now the current values the callback function should write them into this buffer.  ��
 *          ���Э��ջ��Ҫ��ȡ��ǰֵ���ص�����Ӧ�ý�����д�������������
 * \param usAddress 
 *          The starting address of the register.    �Ĵ�������ʼ��ַ��
 * \param usNRegs 
 *          Number of registers to read or write.    Ҫ����д�ļĴ�������
 * \param eMode 
 *			If eMBRegisterMode::MB_REG_WRITE the application register values should be updated from the values in the buffer. 
 *          ���ֵΪ��MB_REG_WRITE��Ӧ�ó���Ĵ�����ֵӦ�ôӻ������е�ֵ���¡�
 *          For example this would be the case when the Modbus master has issued an <b>WRITE SINGLE REGISTER</b> command.
 *          ���磬��Modbus��������<b>WRITE SINGLE REGISTER</b>����ʱ���ͻ�������������
 *   		If the value eMBRegisterMode::MB_REG_READ the application should copy the current values into the buffer \c pucRegBuffer.
 *          ���ֵΪ��MB_REG_READ��Ӧ�ó���Ӧ�ø��Ƶ�ǰֵ��������\c pucRegBuffer��
 *
 * \return 
 *   The function must return one of the following error codes:
 *   - eMBErrorCode::MB_ENOERR 
 *                 If no error occurred. In this case a normal Modbus response is sent.
 *                 ���û�д�����������������£�����һ��������Modbus��Ӧ��
 *   - eMBErrorCode::MB_ENOREG 
 *                 If the application can not supply values for registers within this range.
 *                 ���Ӧ�ó������ڴ˷�Χ��Ϊ�Ĵ����ṩֵ��
 *                 In this case a <b>ILLEGAL DATA ADDRESS</b> exception frame is sent as a response.
 *                 ����������£�����һ�� <b>�Ƿ����ݵ�ַ</b> �쳣֡��Ϊ��Ӧ��
 *   - eMBErrorCode::MB_ETIMEDOUT 
 *                 If the requested register block is currently not available and the application dependent response timeout would be violated. 
 *                 �������ļĴ����鵱ǰ�����ã�����Υ����������Ӧ�ó������Ӧ��ʱ��
 *                 In this case a <b>SLAVE DEVICE BUSY</b> exception is sent as a response.
 *                 ����������£�����һ�� <b>���豸æ</b> �쳣֡��Ϊ��Ӧ��
 *   - eMBErrorCode::MB_EIO 
 *                 If an unrecoverable error occurred. 
 *                 ����������ɻָ��Ĵ���
 *                 In this case a <b>SLAVE DEVICE FAILURE</b> exception is sent as a response.
 *                 ����������£�����һ�� <b>���豸����</b> �쳣֡��Ϊ��Ӧ��
 */
eMBErrorCode    eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress,
                                 USHORT usNRegs, eMBRegisterMode eMode );

/*! \ingroup modbus_registers
 * \brief Callback function used if a <em>Coil Register</em> value is
 *   read or written by the protocol stack. If you are going to use
 *   this function you might use the functions xMBUtilSetBits(  ) and
 *   xMBUtilGetBits(  ) for working with bitfields.
 *
 * \param pucRegBuffer The bits are packed in bytes where the first coil
 *   starting at address \c usAddress is stored in the LSB of the
 *   first byte in the buffer <code>pucRegBuffer</code>.
 *   If the buffer should be written by the callback function unused
 *   coil values (I.e. if not a multiple of eight coils is used) should be set
 *   to zero.
 * \param usAddress The first coil number.
 * \param usNCoils Number of coil values requested.
 * \param eMode If eMBRegisterMode::MB_REG_WRITE the application values should
 *   be updated from the values supplied in the buffer \c pucRegBuffer.
 *   If eMBRegisterMode::MB_REG_READ the application should store the current
 *   values in the buffer \c pucRegBuffer.
 *
 * \return The function must return one of the following error codes:
 *   - eMBErrorCode::MB_ENOERR If no error occurred. In this case a normal
 *       Modbus response is sent.
 *   - eMBErrorCode::MB_ENOREG If the application does not map an coils
 *       within the requested address range. In this case a 
 *       <b>ILLEGAL DATA ADDRESS</b> is sent as a response.
 *   - eMBErrorCode::MB_ETIMEDOUT If the requested register block is
 *       currently not available and the application dependent response
 *       timeout would be violated. In this case a <b>SLAVE DEVICE BUSY</b>
 *       exception is sent as a response.
 *   - eMBErrorCode::MB_EIO If an unrecoverable error occurred. In this case
 *       a <b>SLAVE DEVICE FAILURE</b> exception is sent as a response.
 */
eMBErrorCode    eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress,
                               USHORT usNCoils, eMBRegisterMode eMode );

/*! \ingroup modbus_registers
 * \brief Callback function used if a <em>Input Discrete Register</em> value is
 *   read by the protocol stack.
 *
 * If you are going to use his function you might use the functions
 * xMBUtilSetBits(  ) and xMBUtilGetBits(  ) for working with bitfields.
 *
 * \param pucRegBuffer The buffer should be updated with the current
 *   coil values. The first discrete input starting at \c usAddress must be
 *   stored at the LSB of the first byte in the buffer. If the requested number
 *   is not a multiple of eight the remaining bits should be set to zero.
 * \param usAddress The starting address of the first discrete input.
 * \param usNDiscrete Number of discrete input values.
 * \return The function must return one of the following error codes:
 *   - eMBErrorCode::MB_ENOERR If no error occurred. In this case a normal
 *       Modbus response is sent.
 *   - eMBErrorCode::MB_ENOREG If no such discrete inputs exists.
 *       In this case a <b>ILLEGAL DATA ADDRESS</b> exception frame is sent 
 *       as a response.
 *   - eMBErrorCode::MB_ETIMEDOUT If the requested register block is
 *       currently not available and the application dependent response
 *       timeout would be violated. In this case a <b>SLAVE DEVICE BUSY</b>
 *       exception is sent as a response.
 *   - eMBErrorCode::MB_EIO If an unrecoverable error occurred. In this case
 *       a <b>SLAVE DEVICE FAILURE</b> exception is sent as a response.
 */
eMBErrorCode    eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress,
                                  USHORT usNDiscrete );

eMBErrorCode 
eMBSetSlaveAddr(UCHAR usSalveAddress);
#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
