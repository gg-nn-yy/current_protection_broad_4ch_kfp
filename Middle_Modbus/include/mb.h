/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.  freeemodbus库:一个可移植的Modbus实现Modbus ASCII/RTU。
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

/* 这个头文件中所声明的函数 ：
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
 * 这个模块定义了应用程序的接口。
 * It contains the basic functions and types required to use the Modbus protocol stack.
 * 它包含使用Modbus协议栈所需的基本函数和类型。
 * A typical application will want to call eMBInit() first.
 * 一个典型的应用程序会想要先调用eMBInit()。
 * If the device is ready to answer network requests it must then call eMBEnable() to activate the protocol stack. 
 * 如果设备准备好响应网络请求，那么它必须调用eMBEnableO来激活协议栈。
 * In the main loop the function eMBPoll() must be called periodically. 
 * 在主循环中，必须定期调用eMBPoll()函数。
 * The time interval between pooling depends on the configured Modbus timeout.
 * 池化的时间间隔取决于配置的Modbus超时时间。
 * If an RTOS is available a separate task should be created and the task should always call the function eMBPoll().
 * 如果RTOS可用，则应该创建一个单独的任务，并且该任务应该始终调用eMBPoll()函数。
 * \code
 * // Initialize protocol stack in RTU mode for a slave with address 10 = 0x0A  为地址为10 = OxOA的从机初始化RTU模式的协议栈
 * eMBInit( MB_RTU, 0x0A, 38400, MB_PAR_EVEN );
 * // Enable the Modbus Protocol Stack.
 * eMBEnable(  );
 * for( ;; )
 * {
 *     // Call the main polling loop of the Modbus protocol stack. 调用Modbus协议栈的主轮询循环。
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
 * \brief Modbus serial transmission modes (RTU/ASCII).  Modbus串行传输方式(RTU/ASCII)。
 *
 * Modbus serial supports two transmission modes. Either ASCII or RTU.
 * Modbus串口支持两种传输方式。ASCIl或RTU。
 * RTU is faster but has more hardware requirements and requires a network with a low jitter.
 * RTU速度更快，但对硬件的要求更高，并且需要一个低抖动的网络。
 * ASCII is slower and more reliable on slower links (E.g. modems)
 * ASCIl在较慢的链路(例如调制解调器)上更慢，更可靠。 
 */
    typedef enum
{
    MB_RTU,                     /*!< RTU transmission mode. */
    MB_ASCII,                   /*!< ASCII transmission mode. */
    MB_TCP                      /*!< TCP mode. */
} eMBMode;

/*! \ingroup modbus
 * \brief If register should be written or read.    如果寄存器需要写入或读取。
 *
 * This value is passed to the callback functions which support either reading or writing register values.
 * 这个值被传递给支持读或写寄存器值的回调函数
 * Writing means that the application registers should be updated 
 * and reading means that the modbus protocol stack needs to know the current register values.
 * 写意味着应用程序的寄存器应该被更新,而读则意味着modbus协议栈需要知道当前的寄存器值。
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
 * \brief Errorcodes used by all function in the protocol stack.    协议栈中所有函数使用的错误码。
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
 * 该函数初始化ASCII或RTU模块，并调用移植层的init函数来准备硬件。
 * Please note that the receiver is still disabled and no Modbus frames are processed until eMBEnable( ) has been called.
 * 请注意，接收器仍然处于禁用状态，在调用eMBEnable()之前不会处理任何Modbus帧。
 *
 * \param eMode
 *    If ASCII or RTU mode should be used.    如果使用ASCII或RTU模式。
 * \param ucSlaveAddress
 *    The slave address. Only frames sent to this address or to the broadcast address are processed.
 *    从机地址。只有发送到该地址或广播地址的帧才会被处理。
 * \param ucPort 
 *    The port to use. E.g. 1 for COM1 on windows. 
 *    要使用的端口。例如，1用于windows上的COM1。
 *    This value is platform dependent and some ports simply choose to ignore it.
 *    这个值是平台相关的，一些端口干脆选择忽略它。
 * \param ulBaudRate 
 *    The baudrate. E.g. 19200. Supported baudrates depend on the porting layer.
 *    波特率。例如19200。支持的波特率取决于移植层。
 * \param eParity 
 *    Parity  used for serial transmission.
 *    用于串行传输的校验性。
 *
 * \return 
 *   If no error occurs the function returns eMBErrorCode::MB_ENOERR.
 *   The protocol is then in the disabled state and ready for activation by calling eMBEnable( ). 
 *   然后，协议处于禁用状态，并准备通过调用eMBEnable()来激活。
 *   Otherwise one of the following error codes is returned:
 *    - eMBErrorCode::MB_EINVAL 
 *        If the slave address was not valid. Valid slave addresses are in the range 1 - 247.
 *        如果从地址无效。有效的从属地址在1 - 247范围内。
 *    - eMBErrorCode::MB_EPORTERR 
 *        IF the porting layer returned an error.
 *        如果移植层返回一个错误。
 */
eMBErrorCode    eMBInit( eMBMode eMode, UCHAR ucSlaveAddress,
                         UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity );

/*! \ingroup modbus
 * \brief Initialize the Modbus protocol stack for Modbus TCP.    为Modbus TCP初始化Modbus协议栈。
 *
 * This function initializes the Modbus TCP Module.
 * 初始化Modbus TCP模块。
 * Please note that frame processing is still disabled until eMBEnable( ) is called.
 * 请注意，在调用eMBEnable()之前，帧处理仍然是禁用的。
 *
 * \param usTCPPort The TCP port to listen on.    要侦听的TCP端口。
 * \return 
 *   If the protocol stack has been initialized correctly the function returns eMBErrorCode::MB_ENOERR. 
 *    如果协议栈已经正确初始化，函数返回eMBErrorCode::MB_ENOERR。
 *   Otherwise one of the following error codes is returned:    否则返回以下错误码之一:
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
 *     Modbus协议栈的主池循环。
 *     This function must be called periodically.
 *     这个函数必须定期调用。
 *     The timer interval required is given by the application dependent Modbus slave timeout. 
 *     所需的定时器间隔由依赖于应用程序的Modbus从机超时时间给出。
 *     Internally the function calls xMBPortEventGet() and waits for an event from the receiver or transmitter state machines. 
 *     在内部，该函数调用xMBPortEventGet()并等待来自接收方或发送方状态机的事件。
 *
 * \return 
 *     If the protocol stack is not in the enabled state the function returns eMBErrorCode::MB_EILLSTATE.
 *     如果协议栈未处于启用状态，则返回：MB_EILLSTATE。
 *     Otherwise it returns eMBErrorCode::MB_ENOERR.
 *     否则返回：MB_ENOERR
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
 *   当<em>保持寄存器</em>值被协议栈读取或写入时使用的回调函数
 *   The starting register address is given by \c usAddress and the last register is given by <tt>usAddress + usNRegs - 1</tt>.
 *   起始寄存器地址由\c usAddress给出，末尾寄存器由<tt>usAddress + usNRegs - 1</tt>给出。
 *
 * \param pucRegBuffer
 *			If the application registers values should be updated the buffer points to the new registers values.             写
 *          如果应用程序的寄存器值需要更新，则缓冲区指向新的寄存器值。
 *			If the protocol stack needs to now the current values the callback function should write them into this buffer.  读
 *          如果协议栈需要获取当前值，回调函数应该将它们写入这个缓冲区。
 * \param usAddress 
 *          The starting address of the register.    寄存器的起始地址。
 * \param usNRegs 
 *          Number of registers to read or write.    要读或写的寄存器数。
 * \param eMode 
 *			If eMBRegisterMode::MB_REG_WRITE the application register values should be updated from the values in the buffer. 
 *          如果值为：MB_REG_WRITE，应用程序寄存器的值应该从缓冲区中的值更新。
 *          For example this would be the case when the Modbus master has issued an <b>WRITE SINGLE REGISTER</b> command.
 *          例如，当Modbus主机发出<b>WRITE SINGLE REGISTER</b>命令时，就会出现这种情况。
 *   		If the value eMBRegisterMode::MB_REG_READ the application should copy the current values into the buffer \c pucRegBuffer.
 *          如果值为：MB_REG_READ，应用程序应该复制当前值到缓冲区\c pucRegBuffer。
 *
 * \return 
 *   The function must return one of the following error codes:
 *   - eMBErrorCode::MB_ENOERR 
 *                 If no error occurred. In this case a normal Modbus response is sent.
 *                 如果没有错误发生。在这种情况下，发送一个正常的Modbus响应。
 *   - eMBErrorCode::MB_ENOREG 
 *                 If the application can not supply values for registers within this range.
 *                 如果应用程序不能在此范围内为寄存器提供值。
 *                 In this case a <b>ILLEGAL DATA ADDRESS</b> exception frame is sent as a response.
 *                 在这种情况下，发送一个 <b>非法数据地址</b> 异常帧作为响应。
 *   - eMBErrorCode::MB_ETIMEDOUT 
 *                 If the requested register block is currently not available and the application dependent response timeout would be violated. 
 *                 如果请求的寄存器块当前不可用，并且违反了依赖于应用程序的响应超时。
 *                 In this case a <b>SLAVE DEVICE BUSY</b> exception is sent as a response.
 *                 在这种情况下，发送一个 <b>从设备忙</b> 异常帧作为响应。
 *   - eMBErrorCode::MB_EIO 
 *                 If an unrecoverable error occurred. 
 *                 如果发生不可恢复的错误。
 *                 In this case a <b>SLAVE DEVICE FAILURE</b> exception is sent as a response.
 *                 在这种情况下，发送一个 <b>从设备故障</b> 异常帧作为响应。
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
