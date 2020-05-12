/**
  ******************************************************************************
  * @file    stm32l4xx_ll_dma2d.h
  * @author  MCD Application Team
  * @brief   Header file of DMA2D LL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32L4xx_LL_DMA2D_H
#define STM32L4xx_LL_DMA2D_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx.h"

/** @addtogroup STM32L4xx_LL_Driver
  * @{
  */

#if defined (DMA2D)

/** @defgroup DMA2D_LL DMA2D
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup DMA2D_LL_Private_Macros DMA2D Private Macros
  * @{
  */

/**
  * @}
  */
#endif /*USE_FULL_LL_DRIVER*/

/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_LL_DRIVER)
/** @defgroup DMA2D_LL_ES_Init_Struct DMA2D Exported Init structures
  * @{
  */

/**
  * @brief LL DMA2D Init Structure Definition
  */
typedef struct
{
  uint32_t Mode;                 /*!< Specifies the DMA2D transfer mode.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_MODE.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetMode().*/

  uint32_t ColorMode;            /*!< Specifies the color format of the output image.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_OUTPUT_COLOR_MODE.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColorMode(). */

  uint32_t OutputBlue;           /*!< Specifies the Blue value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */

  uint32_t OutputGreen;          /*!< Specifies the Green value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x3F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */

  uint32_t OutputRed;            /*!< Specifies the Red value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */

  uint32_t OutputAlpha;          /*!< Specifies the Alpha channel of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x01 if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.
                                      - This parameter is not considered if RGB888 or RGB565 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */

  uint32_t OutputMemoryAddress;  /*!< Specifies the memory address.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFFFFFF.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputMemAddr(). */

#if defined(DMA2D_OUTPUT_TWO_BY_TWO_SWAP_SUPPORT)
  uint32_t OutputSwapMode;             /*!< Specifies the output swap mode color format of the output image.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_OUTPUT_SWAP_MODE.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputSwapMode(). */
#endif /* DMA2D_OUTPUT_TWO_BY_TWO_SWAP_SUPPORT */

#if defined(DMA2D_LINE_OFFSET_MODE_SUPPORT)
  uint32_t LineOffsetMode;       /*!< Specifies the output line offset mode.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_LINE_OFFSET_MODE.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetLineOffsetMode(). */
#endif /* DMA2D_LINE_OFFSET_MODE_SUPPORT */

  uint32_t LineOffset;           /*!< Specifies the output line offset value.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF on devices
                                      where the Line Offset Mode feature is available.
                                      else between Min_Data = 0x0000 and Max_Data = 0xFFFF on other devices.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetLineOffset(). */

  uint32_t NbrOfLines;           /*!< Specifies the number of lines of the area to be transferred.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetNbrOfLines(). */

  uint32_t NbrOfPixelsPerLines;  /*!< Specifies the number of pixels per lines of the area to be transfered.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetNbrOfPixelsPerLines(). */

  uint32_t AlphaInversionMode;   /*!< Specifies the output alpha inversion mode.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_ALPHA_INVERSION.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputAlphaInvMode(). */

  uint32_t RBSwapMode;           /*!< Specifies the output Red Blue swap mode.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_RED_BLUE_SWAP.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputRBSwapMode(). */

} LL_DMA2D_InitTypeDef;

/**
  * @brief LL DMA2D Layer Configuration Structure Definition
  */
typedef struct
{
  uint32_t MemoryAddress;        /*!< Specifies the foreground or background memory address.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFFFFFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetMemAddr() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetMemAddr() for background layer. */

  uint32_t LineOffset;           /*!< Specifies the foreground or background line offset value.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetLineOffset() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetLineOffset() for background layer. */

  uint32_t ColorMode;            /*!< Specifies the foreground or background color mode.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_INPUT_COLOR_MODE.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetColorMode() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetColorMode() for background layer. */

  uint32_t CLUTColorMode;        /*!< Specifies the foreground or background CLUT color mode.
                                       - This parameter can be one value of @ref DMA2D_LL_EC_CLUT_COLOR_MODE.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetCLUTColorMode() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetCLUTColorMode() for background layer. */

  uint32_t CLUTSize;             /*!< Specifies the foreground or background CLUT size.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetCLUTSize() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetCLUTSize() for background layer. */

  uint32_t AlphaMode;            /*!< Specifies the foreground or background alpha mode.
                                       - This parameter can be one value of @ref DMA2D_LL_EC_ALPHA_MODE.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetAlphaMode() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetAlphaMode() for background layer. */

  uint32_t Alpha;                /*!< Specifies the foreground or background Alpha value.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetAlpha() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetAlpha() for background layer. */

  uint32_t Blue;                 /*!< Specifies the foreground or background Blue color value.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetBlueColor() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetBlueColor() for background layer. */

  uint32_t Green;                /*!< Specifies the foreground or background Green color value.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetGreenColor() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetGreenColor() for background layer. */

  uint32_t Red;                  /*!< Specifies the foreground or background Red color value.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetRedColor() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetRedColor() for background layer. */

  uint32_t CLUTMemoryAddress;    /*!< Specifies the foreground or background CLUT memory address.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFFFFFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetCLUTMemAddr() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetCLUTMemAddr() for background layer. */

  uint32_t AlphaInversionMode;   /*!< Specifies the foreground or background alpha inversion mode.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_ALPHA_INVERSION.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetAlphaInvMode() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetAlphaInvMode() for background layer. */

  uint32_t RBSwapMode;           /*!< Specifies the foreground or background Red Blue swap mode.
                                      This parameter can be one value of @ref DMA2D_LL_EC_RED_BLUE_SWAP .

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetRBSwapMode() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetRBSwapMode() for background layer. */


} LL_DMA2D_LayerCfgTypeDef;

/**
  * @brief LL DMA2D Output Color Structure Definition
  */
typedef struct
{
  uint32_t ColorMode;            /*!< Specifies the color format of the output image.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_OUTPUT_COLOR_MODE.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColorMode(). */

  uint32_t OutputBlue;           /*!< Specifies the Blue value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */

  uint32_t OutputGreen;          /*!< Specifies the Green value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x3F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */

  uint32_t OutputRed;            /*!< Specifies the Red value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */

  uint32_t OutputAlpha;          /*!< Specifies the Alpha channel of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x01 if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.
                                      - This parameter is not considered if RGB888 or RGB565 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */

} LL_DMA2D_ColorTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DMA2D_LL_Exported_Constants DMA2D Exported Constants
  * @{
  */

/** @defgroup DMA2D_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_DMA2D_ReadReg function
  * @{
  */
#define LL_DMA2D_FLAG_CEIF          DMA2D_ISR_CEIF     /*!< Configuration Error Interrupt Flag */
#define LL_DMA2D_FLAG_CTCIF         DMA2D_ISR_CTCIF    /*!< CLUT Transfer Complete Interrupt Flag */
#define LL_DMA2D_FLAG_CAEIF         DMA2D_ISR_CAEIF    /*!< CLUT Access Error Interrupt Flag */
#define LL_DMA2D_FLAG_TWIF          DMA2D_ISR_TWIF     /*!< Transfer Watermark Interrupt Flag */
#define LL_DMA2D_FLAG_TCIF          DMA2D_ISR_TCIF     /*!< Transfer Complete Interrupt Flag */
#define LL_DMA2D_FLAG_TEIF          DMA2D_ISR_TEIF     /*!< Transfer Error Interrupt Flag */
/**
  * @}
  */

/** @defgroup DMA2D_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_DMA2D_ReadReg and  LL_DMA2D_WriteReg functions
  * @{
  */
#define LL_DMA2D_IT_CEIE             DMA2D_CR_CEIE    /*!< Configuration Error Interrupt */
#define LL_DMA2D_IT_CTCIE            DMA2D_CR_CTCIE   /*!< CLUT Transfer Complete Interrupt */
#define LL_DMA2D_IT_CAEIE            DMA2D_CR_CAEIE   /*!< CLUT Access Error Interrupt */
#define LL_DMA2D_IT_TWIE             DMA2D_CR_TWIE    /*!< Transfer Watermark Interrupt */
#define LL_DMA2D_IT_TCIE             DMA2D_CR_TCIE    /*!< Transfer Complete Interrupt */
#define LL_DMA2D_IT_TEIE             DMA2D_CR_TEIE    /*!< Transfer Error Interrupt */
/**
  * @}
  */

/** @defgroup DMA2D_LL_EC_MODE Mode
  * @{
  */
#define LL_DMA2D_MODE_M2M                       0x00000000U                       /*!< DMA2D memory to memory transfer mode */
#define LL_DMA2D_MODE_M2M_PFC                   DMA2D_CR_MODE_0                   /*!< DMA2D memory to memory with pixel format conversion transfer mode */
#define LL_DMA2D_MODE_M2M_BLEND                 DMA2D_CR_MODE_1                   /*!< DMA2D memory to memory with blending transfer mode */
#define LL_DMA2D_MODE_R2M                       (DMA2D_CR_MODE_0|DMA2D_CR_MODE_1) /*!< DMA2D register to memory transfer mode */
#if defined(DMA2D_M2M_BLEND_FIXED_COLOR_FG_BG_SUPPORT)
#define LL_DMA2D_MODE_M2M_BLEND_FIXED_COLOR_FG  DMA2D_CR_MODE_2                   /*!< DMA2D memory to memory with blending transfer mode and fixed color foreground */
#define LL_DMA2D_MODE_M2M_BLEND_FIXED_COLOR_BG  (DMA2D_CR_MODE_0|DMA2D_CR_MODE_2) /*!< DMA2D memory to memory with blending transfer mode and fixed color background */
#endif /* DMA2D_M2M_BLEND_FIXED_COLOR_FG_BG_SUPPORT */
/**
  * @}
  */

/** @defgroup DMA2D_LL_EC_OUTPUT_COLOR_MODE Output Color Mode
  * @{
  */
#define LL_DMA2D_OUTPUT_MODE_ARGB8888     0x00000000U                           /*!< ARGB8888 */
#define LL_DMA2D_OUTPUT_MODE_RGB888       DMA2D_OPFCCR_CM_0                     /*!< RGB888   */
#define LL_DMA2D_OUTPUT_MODE_RGB565       DMA2D_OPFCCR_CM_1                     /*!< RGB565   */
#define LL_DMA2D_OUTPUT_MODE_ARGB1555     (DMA2D_OPFCCR_CM_0|DMA2D_OPFCCR_CM_1) /*!< ARGB1555 */
#define LL_DMA2D_OUTPUT_MODE_ARGB4444     DMA2D_OPFCCR_CM_2                     /*!< ARGB4444 */
/**
  * @}
  */

/** @defgroup DMA2D_LL_EC_INPUT_COLOR_MODE Input Color Mode
  * @{
  */
#define LL_DMA2D_INPUT_MODE_ARGB8888      0x00000000U                                                /*!< ARGB8888 */
#define LL_DMA2D_INPUT_MODE_RGB888        DMA2D_FGPFCCR_CM_0                                         /*!< RGB888   */
#define LL_DMA2D_INPUT_MODE_RGB565        DMA2D_FGPFCCR_CM_1                                         /*!< RGB565   */
#define LL_DMA2D_INPUT_MODE_ARGB1555      (DMA2D_FGPFCCR_CM_0|DMA2D_FGPFCCR_CM_1)                    /*!< ARGB1555 */
#define LL_DMA2D_INPUT_MODE_ARGB4444      DMA2D_FGPFCCR_CM_2                                         /*!< ARGB4444 */
#define LL_DMA2D_INPUT_MODE_L8            (DMA2D_FGPFCCR_CM_0|DMA2D_FGPFCCR_CM_2)                    /*!< L8       */
#define LL_DMA2D_INPUT_MODE_AL44          (DMA2D_FGPFCCR_CM_1|DMA2D_FGPFCCR_CM_2)                    /*!< AL44     */
#define LL_DMA2D_INPUT_MODE_AL88          (DMA2D_FGPFCCR_CM_0|DMA2D_FGPFCCR_CM_1|DMA2D_FGPFCCR_CM_2) /*!< AL88     */
#define LL_DMA2D_INPUT_MODE_L4            DMA2D_FGPFCCR_CM_3                                         /*!< L4       */
#define LL_DMA2D_INPUT_MODE_A8            (DMA2D_FGPFCCR_CM_0|DMA2D_FGPFCCR_CM_3)                    /*!< A8       */
#define LL_DMA2D_INPUT_MODE_A4            (DMA2D_FGPFCCR_CM_1|DMA2D_FGPFCCR_CM_3)                    /*!< A4       */
/**
  * @}
  */

/** @defgroup DMA2D_LL_EC_ALPHA_MODE Alpha Mode
  * @{
  */
#define LL_DMA2D_ALPHA_MODE_NO_MODIF       0x00000000U             /*!< No modification of the alpha channel value */
#define LL_DMA2D_ALPHA_MODE_REPLACE        DMA2D_FGPFCCR_AM_0      /*!< Replace original alpha channel value by programmed alpha value */
#define LL_DMA2D_ALPHA_MODE_COMBINE        DMA2D_FGPFCCR_AM_1      /*!< Replace original alpha channel value by programmed alpha value
                                                                   with original alpha channel value                              */
/**
  * @}
  */

#if defined(DMA2D_OUTPUT_TWO_BY_TWO_SWAP_SUPPORT)
/** @defgroup DMA2D_LL_EC_OUTPUT_SWAP_MODE Swap Mode
  * @{
  */
#define LL_DMA2D_SWAP_MODE_REGULAR        0x00000000U                      /*!< Regular order */
#define LL_DMA2D_SWAP_MODE_TWO_BY_TWO     DMA2D_OPFCCR_SB                  /*!< Bytes swapped two by two */
/**
  * @}
  */
#endif /* DMA2D_OUTPUT_TWO_BY_TWO_SWAP_SUPPORT */

/** @defgroup DMA2D_LL_EC_RED_BLUE_SWAP Red Blue Swap
  * @{
  */
#define LL_DMA2D_RB_MODE_REGULAR          0x00000000U                      /*!< RGB or ARGB */
#define LL_DMA2D_RB_MODE_SWAP             DMA2D_FGPFCCR_RBS                /*!< BGR or ABGR */
/**
  * @}
  */

/** @defgroup DMA2D_LL_EC_ALPHA_INVERSION Alpha Inversion
  * @{
  */
#define LL_DMA2D_ALPHA_REGULAR          0x00000000U                     /*!< Regular alpha  */
#define LL_DMA2D_ALPHA_INVERTED         DMA2D_FGPFCCR_AI                /*!< Inverted alpha */
/**
  * @}
  */


#if defined(DMA2D_LINE_OFFSET_MODE_SUPPORT)
/** @defgroup DMA2D_LL_EC_LINE_OFFSET_MODE Line Offset Mode
  * @{
  */
#define LL_DMA2D_LINE_OFFSET_PIXELS     0x00000000U                     /*!< Line offsets are expressed in pixels  */
#define LL_DMA2D_LINE_OFFSET_BYTES      DMA2D_CR_LOM                    /*!< Line offsets are expressed in bytes   */
/**
  * @}
  */
#endif /* DMA2D_LINE_OFFSET_MODE_SUPPORT */

/** @defgroup DMA2D_LL_EC_CLUT_COLOR_MODE CLUT Color Mode
  * @{
  */
#define LL_DMA2D_CLUT_COLOR_MODE_ARGB8888          0x00000000U                     /*!< ARGB8888 */
#define LL_DMA2D_CLUT_COLOR_MODE_RGB888            DMA2D_FGPFCCR_CCM               /*!< RGB888   */
/**
  * @}
  */


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DMA2D_LL_Exported_Macros DMA2D Exported Macros
  * @{
  */

/** @defgroup DMA2D_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in DMA2D register.
  * @param  __INSTANCE__ DMA2D Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_DMA2D_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG((__INSTANCE__)->__REG__, (__VALUE__))

/**
  * @brief  Read a value in DMA2D register.
  * @param  __INSTANCE__ DMA2D Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_DMA2D_ReadReg(__INSTANCE__, __REG__) READ_REG((__INSTANCE__)->__REG__)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup DMA2D_LL_Exported_Functions DMA2D Exported Functions
  * @{
  */

/** @defgroup DMA2D_LL_EF_Configuration Configuration Functions
  * @{
  */

/**
  * @brief  Start a DMA2D transfer.
  * @rmtoll CR          START            LL_DMA2D_Start
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_Start(DMA2D_TypeDef *DMA2Dx)
{
  SET_BIT(DMA2Dx->CR, DMA2D_CR_START);
}

/**
  * @brief  Indicate if a DMA2D transfer is ongoing.
  * @rmtoll CR          START            LL_DMA2D_IsTransferOngoing
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsTransferOngoing(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->CR, DMA2D_CR_START) == (DMA2D_CR_START));
}

/**
  * @brief  Suspend DMA2D transfer.
  * @note   This API can be used to suspend automatic foreground or background CLUT loading.
  * @rmtoll CR          SUSP            LL_DMA2D_Suspend
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_Suspend(DMA2D_TypeDef *DMA2Dx)
{
  MODIFY_REG(DMA2Dx->CR, DMA2D_CR_SUSP | DMA2D_CR_START, DMA2D_CR_SUSP);
}

/**
  * @brief  Resume DMA2D transfer.
  * @note   This API can be used to resume automatic foreground or background CLUT loading.
  * @rmtoll CR          SUSP            LL_DMA2D_Resume
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_Resume(DMA2D_TypeDef *DMA2Dx)
{
  CLEAR_BIT(DMA2Dx->CR, DMA2D_CR_SUSP | DMA2D_CR_START);
}

/**
  * @brief  Indicate if DMA2D transfer is suspended.
  * @note   This API can be used to indicate whether or not automatic foreground or
  *         background CLUT loading is suspended.
  * @rmtoll CR          SUSP            LL_DMA2D_IsSuspended
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsSuspenof the following values:
N  *         @arg @ref LL_GPIO_PIN_0
N  *         @arg @ref LL_GPIO_PIN_1
N  *         @arg @ref LL_GPIO_PIN_2
N  *         @arg @ref LL_GPIO_PIN_3
N  *         @arg @ref LL_GPIO_PIN_4
N  *         @arg @ref LL_GPIO_PIN_5
N  *         @arg @ref LL_GPIO_PIN_6
N  *         @arg @ref LL_GPIO_PIN_7
N  *         @arg @ref LL_GPIO_PIN_8
N  *         @arg @ref LL_GPIO_PIN_9
N  *         @arg @ref LL_GPIO_PIN_10
N  *         @arg @ref LL_GPIO_PIN_11
N  *         @arg @ref LL_GPIO_PIN_12
N  *         @arg @ref LL_GPIO_PIN_13
N  *         @arg @ref LL_GPIO_PIN_14
N  *         @arg @ref LL_GPIO_PIN_15
N  * @retval Returned value can be one of the following values:
N  *         @arg @ref LL_GPIO_PULL_NO
N  *         @arg @ref LL_GPIO_PULL_UP
N  *         @arg @ref LL_GPIO_PULL_DOWN
N  */
N__STATIC_INLINE uint32_t LL_GPIO_GetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin)
Xstatic __inline uint32_t LL_GPIO_GetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin)
N{
N  return (uint32_t)(READ_BIT(GPIOx->PUPDR,
N                             (GPIO_PUPDR_PUPDR0 << (POSITION_VAL(Pin) * 2U))) >> (POSITION_VAL(Pin) * 2U));
X  return (uint32_t)(((GPIOx->PUPDR) & (((0x3U << (0U)) << ((__clz(__rbit(Pin))) * 2U)))) >> ((__clz(__rbit(Pin))) * 2U));
N}
N
N/**
N  * @brief  Configure gpio alternate function of a dedicated pin from 0 to 7 for a dedicated port.
N  * @note   Possible values are from AF0 to AF15 depending on target.
N  * @note   Warning: only one pin can be passed as parameter.
N  * @rmtoll AFRL         AFSELy        LL_GPIO_SetAFPin_0_7
N  * @param  GPIOx GPIO Port
N  * @param  Pin This parameter can be one of the following values:
N  *         @arg @ref LL_GPIO_PIN_0
N  *         @arg @ref LL_GPIO_PIN_1
N  *         @arg @ref LL_GPIO_PIN_2
N  *         @arg @ref LL_GPIO_PIN_3
N  *         @arg @ref LL_GPIO_PIN_4
N  *         @arg @ref LL_GPIO_PIN_5
N  *         @arg @ref LL_GPIO_PIN_6
N  *         @arg @ref LL_GPIO_PIN_7
N  * @param  Alternate This parameter can be one of the following values:
N  *         @arg @ref LL_GPIO_AF_0
N  *         @arg @ref LL_GPIO_AF_1
N  *         @arg @ref LL_GPIO_AF_2
N  *         @arg @ref LL_GPIO_AF_3
N  *         @arg @ref LL_GPIO_AF_4
N  *         @arg @ref LL_GPIO_AF_5
N  *         @arg @ref LL_GPIO_AF_6
N  *         @arg @ref LL_GPIO_AF_7
N  *         @arg @ref LL_GPIO_AF_8
N  *         @arg @ref LL_GPIO_AF_9
N  *         @arg @ref LL_GPIO_AF_10
N  *         @arg @ref LL_GPIO_AF_11
N  *         @arg @ref LL_GPIO_AF_12
N  *         @arg @ref LL_GPIO_AF_13
N  *         @arg @ref LL_GPIO_AF_14
N  *         @arg @ref LL_GPIO_AF_15
N  * @retval None
N  */
N__STATIC_INLINE void LL_GPIO_SetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
Xstatic __inline void LL_GPIO_SetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
N{
N  MODIFY_REG(GPIOx->AFR[0], (GPIO_AFRL_AFRL0 << (POSITION_VAL(Pin) * 4U)),
N             (Alternate << (POSITION_VAL(Pin) * 4U)));
X  (((GPIOx->AFR[0])) = ((((((GPIOx->AFR[0]))) & (~(((0xFU << (0U)) << ((__clz(__rbit(Pin))) * 4U))))) | ((Alternate << ((__clz(__rbit(Pin))) * 4U))))));
N}
N
N/**
N  * @brief  Return gpio alternate function of a dedicated pin from 0 to 7 for a dedicated port.
N  * @rmtoll AFRL         AFSELy        LL_GPIO_GetAFPin_0_7
N  * @param  GPIOx GPIO Port
N  * @param  Pin This parameter can be one of the following values:
N  *         @arg @ref LL_GPIO_PIN_0
N  *         @arg @ref LL_GPIO_PIN_1
N  *         @arg @ref LL_GPIO_PIN_2
N  *         @arg @ref LL_GPIO_PIN_3
N  *         @arg @ref LL_GPIO_PIN_4
N  *         @arg @ref LL_GPIO_PIN_5
N  *         @arg @ref LL_GPIO_PIN_6
N  *         @arg @ref LL_GPIO_PIN_7
N  * @retval Returned value can be one of the following values:
N  *         @arg @ref LL_GPIO_AF_0
N  *         @arg @ref LL_GPIO_AF_1
N  *         @arg @ref LL_GPIO_AF_2
N  *         @arg @ref LL_GPIO_AF_3
N  *         @arg @ref LL_GPIO_AF_4
N  *         @arg @ref LL_GPIO_AF_5
N  *         @arg @ref LL_GPIO_AF_6
N  *         @arg @ref LL_GPIO_AF_7
N  *         @arg @ref LL_GPIO_AF_8
N  *         @arg @ref LL_GPIO_AF_9
N  *         @arg @ref LL_GPIO_AF_10
N  *         @arg @ref LL_GPIO_AF_11
N  *         @arg @ref LL_GPIO_AF_12
N  *         @arg @ref LL_GPIO_AF_13
N  *         @arg @ref LL_GPIO_AF_14
N  *         @arg @ref LL_GPIO_AF_15
N  */
N__STATIC_INLINE uint32_t LL_GPIO_GetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin)
Xstatic __inline uint32_t LL_GPIO_GetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin)
N{
N  return (uint32_t)(READ_BIT(GPIOx->AFR[0],
N                             (GPIO_AFRL_AFRL0 << (POSITION_VAL(Pin) * 4U))) >> (POSITION_VAL(Pin) * 4U));
X  return (uint32_t)(((GPIOx->AFR[0]) & (((0xFU << (0U)) << ((__clz(__rbit(Pin))) * 4U)))) >> ((__clz(__rbit(Pin))) * 4U));
N}
N
N/**
N  * @brief  Configure gpio alternate function of a dedicated pin from 8 to 15 for a dedicated port.
N  * @note   Possible values are from AF0 to AF15 depending on target.
N  * @note   Warning: only one pin can be passed as parameter.
N  * @rmtoll AFRH         AFSELy        LL_GPIO_SetAFPin_8_15
N  * @param  GPIOx GPIO Port
N  * @param  Pin This parameter can be one of the following values:
N  *         @arg @ref LL_GPIO_PIN_8
N  *         @arg @ref LL_GPIO_PIN_9
N  *         @arg @ref LL_GPIO_PIN_10
N  *         @arg @ref LL_GPIO_PIN_11
N  *         @arg @ref LL_GPIO_PIN_12
N  *         @arg @ref LL_GPIO_PIN_13
N  *         @arg @ref LL_GPIO_PIN_14
N  *         @arg @ref LL_GPIO_PIN_15
N  * @param  Alternate This parameter can be one of the following values:
N  *         @arg @ref LL_GPIO_AF_0
N  *         @arg @ref LL_GPIO_AF_1
N  *         @arg @ref LL_GPIO_AF_2
N  *         @arg @ref LL_GPIO_AF_3
N  *         @arg @ref LL_GPIO_AF_4
N  *         @arg @ref LL_GPIO_AF_5
N  *         @arg @ref LL_GPIO_AF_6
N  *         @arg @ref LL_GPIO_AF_7
N  *         @arg @ref LL_GPIO_AF_8
N  *         @arg @ref LL_GPIO_AF_9
N  *         @arg @ref LL_GPIO_AF_10
N  *         @arg @ref LL_GPIO_AF_11
N  *         @arg @ref LL_GPIO_AF_12
N  *         @arg @ref LL_GPIO_AF_13
N  *         @arg @ref LL_GPIO_AF_14
N  *         @arg @ref LL_GPIO_AF_15
N  * @retval None
N  */
N__STATIC_INLINE void LL_GPIO_SetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
Xstatic __inline void LL_GPIO_SetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
N{
N  MODIFY_REG(GPIOx->AFR[1], (GPIO_AFRH_AFRH0 << (POSITION_VAL(Pin >> 8U) * 4U)),
N             (Alternate << (POSITION_VAL(Pin >> 8U) * 4U)));
X  (((GPIOx->AFR[1])) = ((((((GPIOx->AFR[1]))) & (~(((0xFU << (0U)) << ((__clz(__rbit(Pin >> 8U))) * 4U))))) | ((Alternate << ((__clz(__rbit(Pin >> 8U))) * 4U))))));
N}
N
N/**
N  * @brief  Return gpio alternate function of a dedicated pin from 8 to 15 for a dedicated port.
N  * @note   Possible values are from AF0 to AF15 depending on target.
N  * @rmtoll AFRH         AFSELy        LL_GPIO_GetAFPin_8_15
N  * @param  GPIOx GPIO Port
N  * @param  Pin This parameter can be one of the following values:
N  *         @arg @ref LL_GPIO_PIN_8
N  *         @arg @ref LL_GPIO_PIN_9
N  *         @arg @ref LL_GPIO_PIN_10
N  *         @arg @ref LL_GPIO_PIN_11
N  *         @arg @ref LL_GPIO_PIN_12
N  *         @arg @ref LL_GPIO_PIN_13
N  *         @arg @ref LL_GPIO_PIN_14
N  *         @arg @ref LL_GPIO_PIN_15
N  * @retval Returned value can be one of the following values:
N  *         @arg @ref LL_GPIO_AF_0
N  *         @arg @ref LL_GPIO_AF_1
N  *         @arg @ref LL_GPIO_AF_2
N  *         @arg @ref LL_GPIO_AF_3
N  *         @arg @ref LL_GPIO_AF_4
N  *         @arg @ref LL_GPIO_AF_5
N  *         @arg @ref LL_GPIO_AF_6
N  *         @arg @ref LL_GPIO_AF_7
N  *         @arg @ref LL_GPIO_AF_8
N  *         @arg @ref LL_GPIO_AF_9
N  *         @arg @ref LL_GPIO_AF_10
N  *         @arg @ref LL_GPIO_AF_11
N  *         @arg @ref LL_GPIO_AF_12
N  *         @arg @ref LL_GPIO_AF_13
N  *         @arg @ref LL_GPIO_AF_14
N  *         @arg @ref LL_GPIO_AF_15
N  */
N__STATIC_INLINE uint32_t LL_GPIO_GetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin)
Xstatic __inline uint32_t LL_GPIO_GetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin)
N{
N  return (uint32_t)(READ_BIT(GPIOx->AFR[1],
N                             (GPIO_AFRH_AFRH0 << (POSITION_VAL(Pin >> 8U) * 4U))) >> (POSITION_VAL(Pin >> 8U) * 4U));
X  return (uint32_t)(((GPIOx->AFR[1]) & (((0xFU << (0U)) << ((__clz(__rbit(Pin >> 8U))) * 4U)))) >> ((__clz(__rbit(Pin >> 8U))) * 4U));
N}
N
N
N/**
N  * @brief  Lock configuration of several pins for a dedicated port.
N  * @note   When the lock sequence has been applied on a port bit, the
N  *         value of this port bit can no longer be modified until the
N  *         next reset.
N  * @note   Each lock bit freezes a specific configuration register
N  *         (control and alternate function registers).
N  * @rmtoll LCKR         LCKK          LL_GPIO_LockPin
N  * @param  GPIOx GPIO Port
N  * @param  PinMask This parameter can be a combination of the following values:
N  *         @arg @ref LL_GPIO_PIN_0
N  *         @arg @ref LL_GPIO_PIN_1
N  *         @arg @ref LL_GPIO_PIN_2
N  *         @arg @ref LL_GPIO_PIN_3
N  *         @arg @ref LL_GPIO_PIN_4
N  *         @arg @ref LL_GPIO_PIN_5
N  *         @arg @ref LL_GPIO_PIN_6
N  *         @arg @ref LL_GPIO_PIN_7
N  *         @arg @ref LL_GPIO_PIN_8
N  *         @arg @ref LL_GPIO_PIN_9
N  *         @arg @ref LL_GPIO_PIN_10
N  *         @arg @ref LL_GPIO_PIN_11
N  *         @arg @ref LL_GPIO_PIN_12
N  *         @arg @ref LL_GPIO_PIN_13
N  *         @arg @ref LL_GPIO_PIN_14
N  *         @arg @ref LL_GPIO_PIN_15
N  *         @arg @ref LL_GPIO_PIN_ALL
N  * @retval None
N  */
N__STATIC_INLINE void LL_GPIO_LockPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
Xstatic __inline void LL_GPIO_LockPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
N{
N  __IO uint32_t temp;
X  volatile uint32_t temp;
N  WRITE_REG(GPIOx->LCKR, GPIO_LCKR_LCKK | PinMask);
X  ((GPIOx->LCKR) = ((0x1U << (16U)) | PinMask));
N  WRITE_REG(GPIOx->LCKR, PinMask);
X  ((GPIOx->LCKR) = (PinMask));
N  WRITE_REG(GPIOx->LCKR, GPIO_LCKR_LCKK | PinMask);
X  ((GPIOx->LCKR) = ((0x1U << (16U)) | PinMask));
N  temp = READ_REG(GPIOx->LCKR);
X  temp = ((GPIOx->LCKR));
N  (void) temp;
N}
N
N/**
N  * @brief  Return 1 if all pins passed as parameter, of a dedicated port, are locked. else Return 0.
N  * @rmtoll LCKR         LCKy          LL_GPIO_IsPinLocked
N  * @param  GPIOx GPIO Port
N  * @param  PinMask This parameter can be a combination of the following values:
N  *         @arg @ref LL_GPIO_PIN_0
N  *         @arg @ref LL_GPIO_PIN_1
N  *         @arg @ref LL_GPIO_PIN_2
N  *         @arg @ref LL_GPIO_PIN_3
N  *         @arg @ref LL_GPIO_PIN_4
N  *         @arg @ref LL_GPIO_PIN_5
N  *         @arg @ref LL_GPIO_PIN_6
N  *         @arg @ref LL_GPIO_PIN_7
N  *         @arg @ref LL_GPIO_PIN_8
N  *         @arg @ref LL_GPIO_PIN_9
N  *         @arg @ref LL_GPIO_PIN_10
N  *         @arg @ref LL_GPIO_PIN_11
N  *         @arg @ref LL_GPIO_PIN_12
N  *         @arg @ref LL_GPIO_PIN_13
N  *         @arg @ref LL_GPIO_PIN_14
N  *         @arg @ref LL_GPIO_PIN_15
N  *         @arg @ref LL_GPIO_PIN_ALL
N  * @retval State of bit (1 or 0).
N  */
N__STATIC_INLINE uint32_t LL_GPIO_IsPinLocked(GPIO_TypeDef *GPIOx, uint32_t PinMask)
Xstatic __inline uint32_t LL_GPIO_IsPinLocked(GPIO_TypeDef *GPIOx, uint32_t PinMask)
N{
N  return (READ_BIT(GPIOx->LCKR, PinMask) == (PinMask));
X  return (((GPIOx->LCKR) & (PinMask)) == (PinMask));
N}
N
N/**
N  * @brief  Return 1 if one of the pin of a dedicated port is locked. else return 0.
N  * @rmtoll LCKR         LCKK          LL_GPIO_IsAnyPinLocked
N  * @param  GPIOx GPIO Port
N  * @retval State of bit (1 or 0).
N  */
N__STATIC_INLINE uint32_t LL_GPIO_IsAnyPinLocked(GPIO_TypeDef *GPIOx)
Xstatic __inline uint32_t LL_GPIO_IsAnyPinLocked(GPIO_TypeDef *GPIOx)
N{
N  return (READ_BIT(GPIOx->LCKR, GPIO_LCKR_LCKK) == (GPIO_LCKR_LCKK));
X  return (((GPIOx->LCKR) & ((0x1U << (16U)))) == ((0x1U << (16U))));
N}
N
N/**
N  * @}
N  */
N
N/** @defgroup GPIO_LL_EF_Data_Access Data Access
N  * @{
N  */
N
N/**
N  * @brief  Return full input data register value for a dedicated port.
N  * @rmtoll IDR          IDy           LL_GPIO_ReadInputPort
N  * @param  GPIOx GPIO Port
N  * @retval Input data register value of port
N  */
N__STATIC_INLINE uint32_t LL_GPIO_ReadInputPort(GPIO_TypeDef *GPIOx)
Xstatic __inline uint32_t LL_GPIO_ReadInputPort(GPIO_TypeDef *GPIOx)
N{
N  return (uint32_t)(READ_REG(GPIOx->IDR));
X  return (uint32_t)(((GPIOx->IDR)));
N}
N
N/**
N  * @brief  Return if input data level for several pins of dedicated port is high or low.
N  * @rmtoll IDR          IDy           LL_GPIO_IsInputPinSet
N  * @param  GPIOx GPIO Port
N  * @param  PinMask This parameter can be a combination of the following values:
N  *         @arg @ref LL_GPIO_PIN_0
N  *         @arg @ref LL_GPIO_PIN_1
N  *         @arg @ref LL_GPIO_PIN_2
N  *         @arg @ref LL_GPIO_PIN_3
N  *         @arg @ref LL_GPIO_PIN_4
N  *         @arg @ref LL_GPIO_PIN_5
N  *         @arg @ref LL_GPIO_PIN_6
N  *         @arg @ref LL_GPIO_PIN_7
N  *         @arg @ref LL_GPIO_PIN_8
N  *         @arg @ref LL_GPIO_PIN_9
N  *         @arg @ref LL_GPIO_PIN_10
N  *         @arg @ref LL_GPIO_PIN_11
N  *         @arg @ref LL_GPIO_PIN_12
N  *         @arg @ref LL_GPIO_PIN_13
N  *         @arg @ref LL_GPIO_PIN_14
N  *         @arg @ref LL_GPIO_PIN_15
N  *         @arg @ref LL_GPIO_PIN_ALL
N  * @retval State of bit (1 or 0).
N  */
N__STATIC_INLINE uint32_t LL_GPIO_IsInputPinSet(GPIO_TypeDef *GPIOx, uint32_t PinMask)
Xstatic __inline uint32_t LL_GPIO_IsInputPinSet(GPIO_TypeDef *GPIOx, uint32_t PinMask)
N{
N  return (READ_BIT(GPIOx->IDR, PinMask) == (PinMask));
X  return (((GPIOx->IDR) & (PinMask)) == (PinMask));
N}
N
N/**
N  * @brief  Write output data register for the port.
N  * @rmtoll ODR          ODy           LL_GPIO_WriteOutputPort
N  * @param  GPIOx GPIO Port
N  * @param  PortValue Level value for each pin of the port
N  * @retval None
N  */
N__STATIC_INLINE void LL_GPIO_WriteOutputPort(GPIO_TypeDef *GPIOx, uint32_t PortValue)
Xstatic __inline void LL_GPIO_WriteOutputPort(GPIO_TypeDef *GPIOx, uint32_t PortValue)
N{
N  WRITE_REG(GPIOx->ODR, PortValue);
X  ((GPIOx->ODR) = (PortValue));
N}
N
N/**
N  * @brief  Return full output data register value for a dedicated port.
N  * @rmtoll ODR          ODy           LL_GPIO_ReadOutputPort
N  * @param  GPIOx GPIO Port
N  * @retval Output data register value of port
N  */
N__STATIC_INLINE uint32_t LL_GPIO_ReadOutputPort(GPIO_TypeDef *GPIOx)
Xstatic __inline uint32_t LL_GPIO_ReadOutputPort(GPIO_TypeDef *GPIOx)
N{
N  return (uint32_t)(READ_REG(GPIOx->ODR));
X  return (uint32_t)(((GPIOx->ODR)));
N}
N
N/**
N  * @brief  Return if input data level for several pins of dedicated port is high or low.
N  * @rmtoll ODR          ODy           LL_GPIO_IsOutputPinSet
N  * @param  GPIOx GPIO Port
N  * @param  PinMask This parameter can be a combination of the following values:
N  *         @arg @ref LL_GPIO_PIN_0
N  *         @arg @ref LL_GPIO_PIN_1
N  *         @arg @ref LL_GPIO_PIN_2
N  *         @arg @ref LL_GPIO_PIN_3
N  *         @arg @ref LL_GPIO_PIN_4
N  *         @arg @ref LL_GPIO_PIN_5
N  *         @arg @ref LL_GPIO_PIN_6
N  *         @arg @ref LL_GPIO_PIN_7
N  *         @arg @ref LL_GPIO_PIN_8
N  *         @arg @ref LL_GPIO_PIN_9
N  *         @arg @ref LL_GPIO_PIN_10
N  *         @arg @ref LL_GPIO_PIN_11
N  *         @arg @ref LL_GPIO_PIN_12
N  *         @arg @ref LL_GPIO_PIN_13
N  *         @arg @ref LL_GPIO_PIN_14
N  *         @arg @ref LL_GPIO_PIN_15
N  *         @arg @ref LL_GPIO_PIN_ALL
N  * @retval State of bit (1 or 0).
N  */
N__STATIC_INLINE uint32_t LL_GPIO_IsOutputPinSet(GPIO_TypeDef *GPIOx, uint32_t PinMask)
Xstatic __inline uint32_t LL_GPIO_IsOutputPinSet(GPIO_TypeDef *GPIOx, uint32_t PinMask)
N{
N  return (READ_BIT(GPIOx->ODR, PinMask) == (PinMask));
X  return (((GPIOx->ODR) & (PinMask)) == (PinMask));
N}
N
N/**
N  * @brief  Set several pins to high level on dedicated gpio port.
N  * @rmtoll BSRR         BSy           LL_GPIO_SetOutputPin
N  * @param  GPIOx GPIO Port
N  * @param  PinMask This parameter can be a combination of the following values:
N  *         @arg @ref LL_GPIO_PIN_0
N  *         @arg @ref LL_GPIO_PIN_1
N  *         @arg @ref LL_GPIO_PIN_2
N  *         @arg @ref LL_GPIO_PIN_3
N  *         @arg @ref LL_GPIO_PIN_4
N  *         @arg @ref LL_GPIO_PIN_5
N  *         @arg @ref LL_GPIO_PIN_6
N  *         @arg @ref LL_GPIO_PIN_7
N  *         @arg @ref LL_GPIO_PIN_8
N  *         @arg @ref LL_GPIO_PIN_9
N  *         @arg @ref LL_GPIO_PIN_10
N  *         @arg @ref LL_GPIO_PIN_11
N  *         @arg @ref LL_GPIO_PIN_12
N  *         @arg @ref LL_GPIO_PIN_13
N  *         @arg @ref LL_GPIO_PIN_14
N  *         @arg @ref LL_GPIO_PIN_15
N  *         @arg @ref LL_GPIO_PIN_ALL
N  * @retval None
N  */
N__STATIC_INLINE void LL_GPIO_SetOutputPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
Xstatic __inline void LL_GPIO_SetOutputPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
N{
N  WRITE_REG(GPIOx->BSRR, PinMask);
X  ((GPIOx->BSRR) = (PinMask));
N}
N
N/**
N  * @brief  Set several pins to low level on dedicated gpio port.
N  * @rmtoll BRR          BRy           LL_GPIO_ResetOutputPin
N  * @param  GPIOx GPIO Port
N  * @param  PinMask This parameter can be a combination of the following values:
N  *         @arg @ref LL_GPIO_PIN_0
N  *         @arg @ref LL_GPIO_PIN_1
N  *         @arg @ref LL_GPIO_PIN_2
N  *         @arg @ref LL_GPIO_PIN_3
N  *         @arg @ref LL_GPIO_PIN_4
N  *         @arg @ref LL_GPIO_PIN_5
N  *         @arg @ref LL_GPIO_PIN_6
N  *         @arg @ref LL_GPIO_PIN_7
N  *         @arg @ref LL_GPIO_PIN_8
N  *         @arg @ref LL_GPIO_PIN_9
N  *         @arg @ref LL_GPIO_PIN_10
N  *         @arg @ref LL_GPIO_PIN_11
N  *         @arg @ref LL_GPIO_PIN_12
N  *         @arg @ref LL_GPIO_PIN_13
N  *         @arg @ref LL_GPIO_PIN_14
N  *         @arg @ref LL_GPIO_PIN_15
N  *         @arg @ref LL_GPIO_PIN_ALL
N  * @retval None
N  */
N__STATIC_INLINE void LL_GPIO_ResetOutputPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
Xstatic __inline void LL_GPIO_ResetOutputPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
N{
N  WRITE_REG(GPIOx->BRR, PinMask);
X  ((GPIOx->BRR) = (PinMask));
N}
N
N/**
N  * @brief  Toggle data value for several pin of dedicated port.
N  * @rmtoll ODR          ODy           LL_GPIO_TogglePin
N  * @param  GPIOx GPIO Port
N  * @param  PinMask This parameter can be a combination of the following values:
N  *         @arg @ref LL_GPIO_PIN_0
N  *         @arg @ref LL_GPIO_PIN_1
N  *         @arg @ref LL_GPIO_PIN_2
N  *         @arg @ref LL_GPIO_PIN_3
N  *         @arg @ref LL_GPIO_PIN_4
N  *         @arg @ref LL_GPIO_PIN_5
N  *         @arg @ref LL_GPIO_PIN_6
N  *         @arg @ref LL_GPIO_PIN_7
N  *         @arg @ref LL_GPIO_PIN_8
N  *         @arg @ref LL_GPIO_PIN_9
N  *         @arg @ref LL_GPIO_PIN_10
N  *         @arg @ref LL_GPIO_PIN_11
N  *         @arg @ref LL_GPIO_PIN_12
N  *         @arg @ref LL_GPIO_PIN_13
N  *         @arg @ref LL_GPIO_PIN_14
N  *         @arg @ref LL_GPIO_PIN_15
N  *         @arg @ref LL_GPIO_PIN_ALL
N  * @retval None
N  */
N__STATIC_INLINE void LL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
Xstatic __inline void LL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
N{
N  WRITE_REG(GPIOx->ODR, READ_REG(GPIOx->ODR) ^ PinMask);
X  ((GPIOx->ODR) = (((GPIOx->ODR)) ^ PinMask));
N}
N
N/**
N  * @}
N  */
N
N#if defined(USE_FULL_LL_DRIVER)
X#if 1L
N/** @defgroup GPIO_LL_EF_Init Initialization and de-initialization functions
N  * @{
N  */
N
NErrorStatus LL_GPIO_DeInit(GPIO_TypeDef *GPIOx);
NErrorStatus LL_GPIO_Init(GPIO_TypeDef *GPIOx, LL_GPIO_InitTypeDef *GPIO_InitStruct);
Nvoid        LL_GPIO_StructInit(LL_GPIO_InitTypeDef *GPIO_InitStruct);
N
N/**
N  * @}
N  */
N#endif /* USE_FULL_LL_DRIVER */
N
N/**
N  * @}
N  */
N
N/**
N  * @}
N  */
N
N#endif /* defined (GPIOA) || defined (GPIOB) || defined (GPIOC) || defined (GPIOD) || defined (GPIOE) || defined (GPIOF) || defined (GPIOG) || defined (GPIOH) */
N/**
N  * @}
N  */
N
N#ifdef __cplusplus
S}
N#endif
N
N#endif /* __STM32F3xx_LL_GPIO_H */
N
N/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
L 24 ".\Line\inc\Line_wrapper_SPL_to_LL.h" 2
N//-----------------------------------------------------------------------------------------------
N
N//----- SPL API ---------------------------------------------------------------------------------
N#include "stm32f30x_exti.h"
L 1 ".\StdPeriph_Library\inc\stm32f30x_exti.h" 1
N/**
N  ******************************************************************************
N  * @file    stm32f30x_exti.h
N  * @author  MCD Application Team
N  * @version V1.2.3
N  * @date    10-July-2015
N  * @brief   This file contains all the functions prototypes for the EXTI 
N  *          firmware library.
N  ******************************************************************************
N  * @attention
N  *
N  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
N  *
N  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
N  * You may not use this file except in compliance with the License.
N  * You may obtain a copy of the License at:
N  *
N  *        http://www.st.com/software_license_agreement_liberty_v2
N  *
N  * Unless required by applicable law or agreed to in writing, software 
N  * distributed under the License is distributed on an "AS IS" BASIS, 
N  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
N  * See the License for the specific language governing permissions and
N  * limitations under the License.
N  *
N  ******************************************************************************
N  */ 
N
N/* Define to prevent recursive inclusion -------------------------------------*/
N#ifndef __STM32F30x_EXTI_H
N#define __STM32F30x_EXTI_H
N
N#ifdef __cplusplus
S extern "C" {
N#endif
N
N/* Includes ------------------------------------------------------------------*/
N#include "stm32f3xx.h"
N
N/** @addtogroup STM32F30x_StdPeriph_Driver
N  * @{
N  */
N
N/** @addtogroup EXTI
N  * @{
N  */
N
N/* Exported types ------------------------------------------------------------*/
N
N/** 
N  * @brief  EXTI mode enumeration  
N  */
N
Ntypedef enum
N{
N  EXTI_Mode_Interrupt = 0x00,
N  EXTI_Mode_Event = 0x04
N}EXTIMode_TypeDef;
N
N#define IS_EXTI_MODE(MODE) (((MODE) == EXTI_Mode_Interrupt) || ((MODE) == EXTI_Mode_Event))
N
N/** 
N  * @brief  EXTI Trigger enumeration  
N  */
N
Ntypedef enum
N{
N  EXTI_Trigger_Rising = 0x08,
N  EXTI_Trigger_Falling = 0x0C,
N  EXTI_Trigger_Rising_Falling = 0x10
N}EXTITrigger_TypeDef;
N
N#define IS_EXTI_TRIGGER(TRIGGER) (((TRIGGER) == EXTI_Trigger_Rising) || \
N                                  ((TRIGGER) == EXTI_Trigger_Falling) || \
N                                  ((TRIGGER) == EXTI_Trigger_Rising_Falling))
X#define IS_EXTI_TRIGGER(TRIGGER) (((TRIGGER) == EXTI_Trigger_Rising) ||                                   ((TRIGGER) == EXTI_Trigger_Falling) ||                                   ((TRIGGER) == EXTI_Trigger_Rising_Falling))
N/** 
N  * @brief  EXTI Init Structure definition  
N  */
N
Ntypedef struct
N{
N  uint32_t EXTI_Line;               /*!< Specifies the EXTI lines to be enabled or disabled.
N                                         This parameter can be any combination of @ref EXTI_Lines */
N   
N  EXTIMode_TypeDef EXTI_Mode;       /*!< Specifies the mode for the EXTI lines.
N                                         This parameter can be a value of @ref EXTIMode_TypeDef */
N
N  EXTITrigger_TypeDef EXTI_Trigger; /*!< Specifies the trigger signal active edge for the EXTI lines.
N                                         This parameter can be a value of @ref EXTITrigger_TypeDef */
N
N  FunctionalState EXTI_LineCmd;     /*!< Specifies the new state of the selected EXTI lines.
N                                         This parameter can be set either to ENABLE or DISABLE */
N}EXTI_InitTypeDef;
N
N/* Exported constants --------------------------------------------------------*/
N
N/** @defgroup EXTI_Exported_Constants
N  * @{
N  */ 
N/** @defgroup EXTI_Lines 
N  * @{
N  */
N
N#define EXTI_Line0       ((uint32_t)0x00)  /*!< External interrupt line 0  */
N#define EXTI_Line1       ((uint32_t)0x01)  /*!< External interrupt line 1  */
N#define EXTI_Line2       ((uint32_t)0x02)  /*!< External interrupt line 2  */
N#define EXTI_Line3       ((uint32_t)0x03)  /*!< External interrupt line 3  */
N#define EXTI_Line4       ((uint32_t)0x04)  /*!< External interrupt line 4  */
N#define EXTI_Line5       ((uint32_t)0x05)  /*!< External interrupt line 5  */
N#define EXTI_Line6       ((uint32_t)0x06)  /*!< External interrupt line 6  */
N#define EXTI_Line7       ((uint32_t)0x07)  /*!< External interrupt line 7  */
N#define EXTI_Line8       ((uint32_t)0x08)  /*!< External interrupt line 8  */
N#define EXTI_Line9       ((uint32_t)0x09)  /*!< External interrupt line 9  */
N#define EXTI_Line10      ((uint32_t)0x0A)  /*!< External interrupt line 10 */
N#define EXTI_Line11      ((uint32_t)0x0B)  /*!< External interrupt line 11 */
N#define EXTI_Line12      ((uint32_t)0x0C)  /*!< External interrupt line 12 */
N#define EXTI_Line13      ((uint32_t)0x0D)  /*!< External interrupt line 13 */
N#define EXTI_Line14      ((uint32_t)0x0E)  /*!< External interrupt line 14 */
N#define EXTI_Line15      ((uint32_t)0x0F)  /*!< External interrupt line 15 */
N#define EXTI_Line16      ((uint32_t)0x10)  /*!< External interrupt line 16 
N                                                      Connected to the PVD Output */
N#define EXTI_Line17      ((uint32_t)0x11)  /*!< Internal interrupt line 17 
N                                                      Connected to the RTC Alarm 
N                                                      event */
N#define EXTI_Line18      ((uint32_t)0x12)  /*!< Internal interrupt line 18 
N                                                      Connected to the USB Device
N                                                      Wakeup from suspend event */
N#define EXTI_Line19      ((uint32_t)0x13)  /*!< Internal interrupt line 19
N                                                      Connected to the RTC Tamper
N                                                      and Time Stamp events */
N#define EXTI_Line20      ((uint32_t)0x14)  /*!< Internal interrupt line 20
N                                                      Connected to the RTC wakeup
N                                                      event */                                                      
N#define EXTI_Line21      ((uint32_t)0x15)  /*!< Internal interrupt line 21
N                                                      Connected to the Comparator 1
N                                                      event */
N#define EXTI_Line22      ((uint32_t)0x16)  /*!< Internal interrupt line 22
N                                                      Connected to the Comparator 2
N                                                      event */
N#define EXTI_Line23      ((uint32_t)0x17)  /*!< Internal interrupt line 23
N                                                      Connected to the I2C1 wakeup
N                                                      event */
N#define EXTI_Line24      ((uint32_t)0x18)  /*!< Internal interrupt line 24
N                                                      Connected to the I2C2 wakeup
N                                                      event */
N#define EXTI_Line25      ((uint32_t)0x19)  /*!< Internal interrupt line 25
N                                                      Connected to the USART1 wakeup
N                                                      event */
N#define EXTI_Line26      ((uint32_t)0x1A)  /*!< Internal interrupt line 26
N                                                      Connected to the USART2 wakeup
N                                                      event */
N#define EXTI_Line27      ((uint32_t)0x1B)  /*!< Internal interrupt line 27
N                                                       reserved */
N#define EXTI_Line28      ((uint32_t)0x1C)  /*!< Internal interrupt line 28
N                                                      Connected to the USART3 wakeup
N                                                      event */
N#define EXTI_Line29      ((uint32_t)0x1D)  /*!< Internal interrupt line 29
N                                                      Connected to the Comparator 3 
N                                                      event */
N#define EXTI_Line30      ((uint32_t)0x1E)  /*!< Internal interrupt line 30
N                                                      Connected to the Comparator 4 
N                                                      event */
N#define EXTI_Line31      ((uint32_t)0x1F)  /*!< Internal interrupt line 31
N                                                      Connected to the Comparator 5 
N                                                      event */
N#define EXTI_Line32      ((uint32_t)0x20)  /*!< Internal interrupt line 32
N                                                      Connected to the Comparator 6 
N                                                      event */
N#define EXTI_Line33      ((uint32_t)0x21)  /*!< Internal interrupt line 33
N                                                      Connected to the Comparator 7 
N                                                      event */
N#define EXTI_Line34      ((uint32_t)0x22)  /*!< Internal interrupt line 34
N                                                      Connected to the USART4 wakeup
N                                                      event */
N#define EXTI_Line35      ((uint32_t)0x23)  /*!< Internal interrupt line 35
N                                                      Connected to the USART5 wakeup
N                                                      event */
N                                                                                                                                                                                                                                                                                                                                                                                                                                                
N#define IS_EXTI_LINE_ALL(LINE) ((LINE) <= 0x23)
N#define IS_EXTI_LINE_EXT(LINE) (((LINE) <= 0x16) || (((LINE) == EXTI_Line29) || ((LINE) == EXTI_Line30) || \
N                               ((LINE) == EXTI_Line31) || ((LINE) == EXTI_Line32) || ((LINE) == EXTI_Line33)))
X#define IS_EXTI_LINE_EXT(LINE) (((LINE) <= 0x16) || (((LINE) == EXTI_Line29) || ((LINE) == EXTI_Line30) ||                                ((LINE) == EXTI_Line31) || ((LINE) == EXTI_Line32) || ((LINE) == EXTI_Line33)))
N
N#define IS_GET_EXTI_LINE(LINE) (((LINE) == EXTI_Line0) || ((LINE) == EXTI_Line1) || \
N                                ((LINE) == EXTI_Line2) || ((LINE) == EXTI_Line3) || \
N                                ((LINE) == EXTI_Line4) || ((LINE) == EXTI_Line5) || \
N                                ((LINE) == EXTI_Line6) || ((LINE) == EXTI_Line7) || \
N                                ((LINE) == EXTI_Line8) || ((LINE) == EXTI_Line9) || \
N                                ((LINE) == EXTI_Line10) || ((LINE) == EXTI_Line11) || \
N                                ((LINE) == EXTI_Line12) || ((LINE) == EXTI_Line13) || \
N                                ((LINE) == EXTI_Line14) || ((LINE) == EXTI_Line15) || \
N                                ((LINE) == EXTI_Line16) || ((LINE) == EXTI_Line17) || \
N                                ((LINE) == EXTI_Line18) || ((LINE) == EXTI_Line19) || \
N                                ((LINE) == EXTI_Line20) || ((LINE) == EXTI_Line21) || \
N                                ((LINE) == EXTI_Line22) || ((LINE) == EXTI_Line29) || \
N                                ((LINE) == EXTI_Line30) || ((LINE) == EXTI_Line31) || \
N                                ((LINE) == EXTI_Line32) || ((LINE) == EXTI_Line33))
X#define IS_GET_EXTI_LINE(LINE) (((LINbrief  Set DMA2D background color mode.
  * @rmtoll BGPFCCR          CM          LL_DMA2D_BGND_SetColorMode
  * @param  DMA2Dx DMA2D Instance
  * @param  ColorMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_INPUT_MODE_RGB888
  *         @arg @ref LL_DMA2D_INPUT_MODE_RGB565
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB1555
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB4444
  *         @arg @ref LL_DMA2D_INPUT_MODE_L8
  *         @arg @ref LL_DMA2D_INPUT_MODE_AL44
  *         @arg @ref LL_DMA2D_INPUT_MODE_AL88
  *         @arg @ref LL_DMA2D_INPUT_MODE_L4
  *         @arg @ref LL_DMA2D_INPUT_MODE_A8
  *         @arg @ref LL_DMA2D_INPUT_MODE_A4
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetColorMode(DMA2D_TypeDef *DMA2Dx, uint32_t ColorMode)
{
  MODIFY_REG(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_CM, ColorMode);
}

/**
  * @brief  Return DMA2D background color mode.
  * @rmtoll BGPFCCR          CM          LL_DMA2D_BGND_GetColorMode
  * @param  DMA2Dx DMA2D Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_INPUT_MODE_RGB888
  *         @arg @ref LL_DMA2D_INPUT_MODE_RGB565
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB1555
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB4444
  *         @arg @ref LL_DMA2D_INPUT_MODE_L8
  *         @arg @ref LL_DMA2D_INPUT_MODE_AL44
  *         @arg @ref LL_DMA2D_INPUT_MODE_AL88
  *         @arg @ref LL_DMA2D_INPUT_MODE_L4
  *         @arg @ref LL_DMA2D_INPUT_MODE_A8
  *         @arg @ref LL_DMA2D_INPUT_MODE_A4
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetColorMode(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_CM));
}

/**
  * @brief  Set DMA2D background alpha mode.
  * @rmtoll BGPFCCR          AM         LL_DMA2D_BGND_SetAlphaMode
  * @param  DMA2Dx DMA2D Instance
  * @param  AphaMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_ALPHA_MODE_NO_MODIF
  *         @arg @ref LL_DMA2D_ALPHA_MODE_REPLACE
  *         @arg @ref LL_DMA2D_ALPHA_MODE_COMBINE
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetAlphaMode(DMA2D_TypeDef *DMA2Dx, uint32_t AphaMode)
{
  MODIFY_REG(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_AM, AphaMode);
}

/**
  * @brief  Return DMA2D background alpha mode.
  * @rmtoll BGPFCCR          AM          LL_DMA2D_BGND_GetAlphaMode
  * @param  DMA2Dx DMA2D Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA2D_ALPHA_MODE_NO_MODIF
  *         @arg @ref LL_DMA2D_ALPHA_MODE_REPLACE
  *         @arg @ref LL_DMA2D_ALPHA_MODE_COMBINE
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetAlphaMode(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_AM));
}

/**
  * @brief  Set DMA2D background alpha value, expressed on 8 bits ([7:0] bits).
  * @rmtoll BGPFCCR          ALPHA         LL_DMA2D_BGND_SetAlpha
  * @param  DMA2Dx DMA2D Instance
  * @param  Alpha Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetAlpha(DMA2D_TypeDef *DMA2Dx, uint32_t Alpha)
{
  MODIFY_REG(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_ALPHA, (Alpha << DMA2D_BGPFCCR_ALPHA_Pos));
}

/**
  * @brief  Return DMA2D background alpha value, expressed on 8 bits ([7:0] bits).
  * @rmtoll BGPFCCR          ALPHA          LL_DMA2D_BGND_GetAlpha
  * @param  DMA2Dx DMA2D Instance
  * @retval Alpha value between Min_Data=0 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetAlpha(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_ALPHA) >> DMA2D_BGPFCCR_ALPHA_Pos);
}

/**
  * @brief  Set DMA2D background Red Blue swap mode.
  * @rmtoll BGPFCCR          RBS         LL_DMA2D_BGND_SetRBSwapMode
  * @param  DMA2Dx DMA2D Instance
  * @param  RBSwapMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_RB_MODE_REGULAR
  *         @arg @ref LL_DMA2D_RB_MODE_SWAP
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetRBSwapMode(DMA2D_TypeDef *DMA2Dx, uint32_t RBSwapMode)
{
  MODIFY_REG(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_RBS, RBSwapMode);
}

/**
  * @brief  Return DMA2D background Red Blue swap mode.
  * @rmtoll BGPFCCR          RBS          LL_DMA2D_BGND_GetRBSwapMode
  * @param  DMA2Dx DMA2D Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA2D_RB_MODE_REGULAR
  *         @arg @ref LL_DMA2D_RB_MODE_SWAP
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetRBSwapMode(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_RBS));
}

/**
  * @brief  Set DMA2D background alpha inversion mode.
  * @rmtoll BGPFCCR          AI         LL_DMA2D_BGND_SetAlphaInvMode
  * @param  DMA2Dx DMA2D Instance
  * @param  AlphaInversionMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_ALPHA_REGULAR
  *         @arg @ref LL_DMA2D_ALPHA_INVERTED
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetAlphaInvMode(DMA2D_TypeDef *DMA2Dx, uint32_t AlphaInversionMode)
{
  MODIFY_REG(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_AI, AlphaInversionMode);
}

/**
  * @brief  Return DMA2D background alpha inversion mode.
  * @rmtoll BGPFCCR          AI          LL_DMA2D_BGND_GetAlphaInvMode
  * @param  DMA2Dx DMA2D Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA2D_ALPHA_REGULAR
  *         @arg @ref LL_DMA2D_ALPHA_INVERTED
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetAlphaInvMode(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_AI));
}

/**
  * @brief  Set DMA2D background line offset, expressed on 14 bits ([13:0] bits).
  * @rmtoll BGOR          LO         LL_DMA2D_BGND_SetLineOffset
  * @param  DMA2Dx DMA2D Instance
  * @param  LineOffset Value between Min_Data=0 and Max_Data=0x3FF
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetLineOffset(DMA2D_TypeDef *DMA2Dx, uint32_t LineOffset)
{
  MODIFY_REG(DMA2Dx->BGOR, DMA2D_BGOR_LO, LineOffset);
}

/**
  * @brief  Return DMA2D background line offset, expressed on 14 bits ([13:0] bits).
  * @rmtoll BGOR          LO          LL_DMA2D_BGND_GetLineOffset
  * @param  DMA2Dx DMA2D Instance
  * @retval Background line offset value between Min_Data=0 and Max_Data=0x3FF
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetLineOffset(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGOR, DMA2D_BGOR_LO));
}

/**
  * @brief  Set DMA2D background color values, expressed on 24 bits ([23:0] bits).
  * @rmtoll BGCOLR          RED          LL_DMA2D_BGND_SetColor
  * @rmtoll BGCOLR          GREEN        LL_DMA2D_BGND_SetColor
  * @rmtoll BGCOLR          BLUE         LL_DMA2D_BGND_SetColor
  * @param  DMA2Dx DMA2D Instance
  * @param  Red   Value between Min_Data=0 and Max_Data=0xFF
  * @param  Green Value between Min_Data=0 and Max_Data=0xFF
  * @param  Blue  Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetColor(DMA2D_TypeDef *DMA2Dx, uint32_t Red, uint32_t Green, uint32_t Blue)
{
  MODIFY_REG(DMA2Dx->BGCOLR, (DMA2D_BGCOLR_RED | DMA2D_BGCOLR_GREEN | DMA2D_BGCOLR_BLUE), \
             ((Red << DMA2D_BGCOLR_RED_Pos) | (Green << DMA2D_BGCOLR_GREEN_Pos) | Blue));
}

/**
  * @brief  Set DMA2D background red color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll BGCOLR          RED         LL_DMA2D_BGND_SetRedColor
  * @param  DMA2Dx DMA2D Instance
  * @param  Red Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetRedColor(DMA2D_TypeDef *DMA2Dx, uint32_t Red)
{
  MODIFY_REG(DMA2Dx->BGCOLR, DMA2D_BGCOLR_RED, (Red << DMA2D_BGCOLR_RED_Pos));
}

/**
  * @brief  Return DMA2D background red color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll BGCOLR          RED          LL_DMA2D_BGND_GetRedColor
  * @param  DMA2Dx DMA2D Instance
  * @retval Red color value between Min_Data=0 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetRedColor(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGCOLR, DMA2D_BGCOLR_RED) >> DMA2D_BGCOLR_RED_Pos);
}

/**
  * @brief  Set DMA2D background green color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll BGCOLR          GREEN         LL_DMA2D_BGND_SetGreenColor
  * @param  DMA2Dx DMA2D Instance
  * @param  Green Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetGreenColor(DMA2D_TypeDef *DMA2Dx, uint32_t Green)
{
  MODIFY_REG(DMA2Dx->BGCOLR, DMA2D_BGCOLR_GREEN, (Green << DMA2D_BGCOLR_GREEN_Pos));
}

/**
  * @brief  Return DMA2D background green color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll BGCOLR          GREEN          LL_DMA2D_BGND_GetGreenColor
  * @param  DMA2Dx DMA2D Instance
  * @retval Green color value between Min_Data=0 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetGreenColor(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGCOLR, DMA2D_BGCOLR_GREEN) >> DMA2D_BGCOLR_GREEN_Pos);
}

/**
  * @brief  Set DMA2D background blue color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll BGCOLR          BLUE         LL_DMA2D_BGND_SetBlueColor
  * @param  DMA2Dx DMA2D Instance
  * @param  Blue Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetBlueColor(DMA2D_TypeDef *DMA2Dx, uint32_t Blue)
{
  MODIFY_REG(DMA2Dx->BGCOLR, DMA2D_BGCOLR_BLUE, Blue);
}

/**
  * @brief  Return DMA2D background blue color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll BGCOLR          BLUE          LL_DMA2D_BGND_GetBlueColor
  * @param  DMA2Dx DMA2D Instance
  * @retval Blue color value between Min_Data=0 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetBlueColor(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGCOLR, DMA2D_BGCOLR_BLUE));
}

/**
  * @brief  Set DMA2D background CLUT memory address, expressed on 32 bits ([31:0] bits).
  * @rmtoll BGCMAR          MA         LL_DMA2D_BGND_SetCLUTMemAddr
  * @param  DMA2Dx DMA2D Instance
  * @param  CLUTMemoryAddress Value between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetCLUTMemAddr(DMA2D_TypeDef *DMA2Dx, uint32_t CLUTMemoryAddress)
{
  LL_DMA2D_WriteReg(DMA2Dx, BGCMAR, CLUTMemoryAddress);
}

/**
  * @brief  Get DMA2D background CLUT memory address, expressed on 32 bits ([31:0] bits).
  * @rmtoll BGCMAR          MA           LL_DMA2D_BGND_GetCLUTMemAddr
  * @param  DMA2Dx DMA2D Instance
  * @retval Background CLUT memory address value between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetCLUTMemAddr(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(LL_DMA2D_ReadReg(DMA2Dx, BGCMAR));
}

/**
  * @brief  Set DMA2D background CLUT size, expressed on 8 bits ([7:0] bits).
  * @rmtoll BGPFCCR          CS         LL_DMA2D_BGND_SetCLUTSize
  * @param  DMA2Dx DMA2D Instance
  * @param  CLUTSize Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetCLUTSize(DMA2D_TypeDef *DMA2Dx, uint32_t CLUTSize)
{
  MODIFY_REG(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_CS, (CLUTSize << DMA2D_BGPFCCR_CS_Pos));
}

/**
  * @brief  Get DMA2D background CLUT size, expressed on 8 bits ([7:0] bits).
  * @rmtoll BGPFCCR          CS           LL_DMA2D_BGND_GetCLUTSize
  * @param  DMA2Dx DMA2D Instance
  * @retval Background CLUT size value between Min_Data=0 and Max_Data=0xFF
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetCLUTSize(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_CS) >> DMA2D_BGPFCCR_CS_Pos);
}

/**
  * @brief  Set DMA2D background CLUT color mode.
  * @rmtoll BGPFCCR          CCM         LL_DMA2D_BGND_SetCLUTColorMode
  * @param  DMA2Dx DMA2D Instance
  * @param  CLUTColorMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_CLUT_COLOR_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_CLUT_COLOR_MODE_RGB888
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_BGND_SetCLUTColorMode(DMA2D_TypeDef *DMA2Dx, uint32_t CLUTColorMode)
{
  MODIFY_REG(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_CCM, CLUTColorMode);
}

/**
  * @brief  Return DMA2D background CLUT color mode.
  * @rmtoll BGPFCCR          CCM          LL_DMA2D_BGND_GetCLUTColorMode
  * @param  DMA2Dx DMA2D Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA2D_CLUT_COLOR_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_CLUT_COLOR_MODE_RGB888
  */
__STATIC_INLINE uint32_t  LL_DMA2D_BGND_GetCLUTColorMode(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->BGPFCCR, DMA2D_BGPFCCR_CCM));
}

/**
  * @}
  */

/**
  * @}
  */


/** @defgroup DMA2D_LL_EF_FLAG_MANAGEMENT Flag Management
  * @{
  */

/**
  * @brief  Check if the DMA2D Configuration Error Interrupt Flag is set or not
  * @rmtoll ISR          CEIF            LL_DMA2D_IsActiveFlag_CE
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsActiveFlag_CE(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->ISR, DMA2D_ISR_CEIF) == (DMA2D_ISR_CEIF));
}

/**
  * @brief  Check if the DMA2D CLUT Transfer Complete Interrupt Flag is set or not
  * @rmtoll ISR          CTCIF            LL_DMA2D_IsActiveFlag_CTC
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsActiveFlag_CTC(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->ISR, DMA2D_ISR_CTCIF) == (DMA2D_ISR_CTCIF));
}

/**
  * @brief  Check if the DMA2D CLUT Access Error Interrupt Flag is set or not
  * @rmtoll ISR          CAEIF            LL_DMA2D_IsActiveFlag_CAE
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsActiveFlag_CAE(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->ISR, DMA2D_ISR_CAEIF) == (DMA2D_ISR_CAEIF));
}

/**
  * @brief  Check if the DMA2D Transfer Watermark Interrupt Flag is set or not
  * @rmtoll ISR          TWIF            LL_DMA2D_IsActiveFlag_TW
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsActiveFlag_TW(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->ISR, DMA2D_ISR_TWIF) == (DMA2D_ISR_TWIF));
}

/**
  * @brief  Check if the DMA2D Transfer Complete Interrupt Flag is set or not
  * @rmtoll ISR          TCIF            LL_DMA2D_IsActiveFlag_TC
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsActiveFlag_TC(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->ISR, DMA2D_ISR_TCIF) == (DMA2D_ISR_TCIF));
}

/**
  * @brief  Check if the DMA2D Transfer Error Interrupt Flag is set or not
  * @rmtoll ISR          TEIF            LL_DMA2D_IsActiveFlag_TE
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsActiveFlag_TE(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->ISR, DMA2D_ISR_TEIF) == (DMA2D_ISR_TEIF));
}

/**
  * @brief  Clear DMA2D Configuration Error Interrupt Flag
  * @rmtoll IFCR          CCEIF          LL_DMA2D_ClearFlag_CE
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_ClearFlag_CE(DMA2D_TypeDef *DMA2Dx)
{
  WRITE_REG(DMA2Dx->IFCR, DMA2D_IFCR_CCEIF);
}

/**
  * @brief  Clear DMA2D CLUT Transfer Complete Interrupt Flag
  * @rmtoll IFCR          CCTCIF          LL_DMA2D_ClearFlag_CTC
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_ClearFlag_CTC(DMA2D_TypeDef *DMA2Dx)
{
  WRITE_REG(DMA2Dx->IFCR, DMA2D_IFCR_CCTCIF);
}

/**
  * @brief  Clear DMA2D CLUT Access Error Interrupt Flag
  * @rmtoll IFCR          CAECIF          LL_DMA2D_ClearFlag_CAE
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_ClearFlag_CAE(DMA2D_TypeDef *DMA2Dx)
{
  WRITE_REG(DMA2Dx->IFCR, DMA2D_IFCR_CAECIF);
}

/**
  * @brief  Clear DMA2D Transfer Watermark Interrupt Flag
  * @rmtoll IFCR          CTWIF          LL_DMA2D_ClearFlag_TW
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_ClearFlag_TW(DMA2D_TypeDef *DMA2Dx)
{
  WRITE_REG(DMA2Dx->IFCR, DMA2D_IFCR_CTWIF);
}

/**
  * @brief  Clear DMA2D Transfer Complete Interrupt Flag
  * @rmtoll IFCR          CTCIF          LL_DMA2D_ClearFlag_TC
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_ClearFlag_TC(DMA2D_TypeDef *DMA2Dx)
{
  WRITE_REG(DMA2Dx->IFCR, DMA2D_IFCR_CTCIF);
}

/**
  * @brief  Clear DMA2D Transfer Error Interrupt Flag
  * @rmtoll IFCR          CTEIF          LL_DMA2D_ClearFlag_TE
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_ClearFlag_TE(DMA2D_TypeDef *DMA2Dx)
{
  WRITE_REG(DMA2Dx->IFCR, DMA2D_IFCR_CTEIF);
}

/**
  * @}
  */

/** @defgroup DMA2D_LL_EF_IT_MANAGEMENT Interruption Management
  * @{
  */

/**
  * @brief  Enable Configuration Error Interrupt
  * @rmtoll CR          CEIE        LL_DMA2D_EnableIT_CE
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_EnableIT_CE(DMA2D_TypeDef *DMA2Dx)
{
  SET_BIT(DMA2Dx->CR, DMA2D_CR_CEIE);
}

/**
  * @brief  Enable CLUT Transfer Complete Interrupt
  * @rmtoll CR          CTCIE        LL_DMA2D_EnableIT_CTC
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_EnableIT_CTC(DMA2D_TypeDef *DMA2Dx)
{
  SET_BIT(DMA2Dx->CR, DMA2D_CR_CTCIE);
}

/**
  * @brief  Enable CLUT Access Error Interrupt
  * @rmtoll CR          CAEIE        LL_DMA2D_EnableIT_CAE
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_EnableIT_CAE(DMA2D_TypeDef *DMA2Dx)
{
  SET_BIT(DMA2Dx->CR, DMA2D_CR_CAEIE);
}

/**
  * @brief  Enable Transfer Watermark Interrupt
  * @rmtoll CR          TWIE        LL_DMA2D_EnableIT_TW
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_EnableIT_TW(DMA2D_TypeDef *DMA2Dx)
{
  SET_BIT(DMA2Dx->CR, DMA2D_CR_TWIE);
}

/**
  * @brief  Enable Transfer Complete Interrupt
  * @rmtoll CR          TCIE        LL_DMA2D_EnableIT_TC
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_EnableIT_TC(DMA2D_TypeDef *DMA2Dx)
{
  SET_BIT(DMA2Dx->CR, DMA2D_CR_TCIE);
}

/**
  * @brief  Enable Transfer Error Interrupt
  * @rmtoll CR          TEIE        LL_DMA2D_EnableIT_TE
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_EnableIT_TE(DMA2D_TypeDef *DMA2Dx)
{
  SET_BIT(DMA2Dx->CR, DMA2D_CR_TEIE);
}

/**
  * @brief  Disable Configuration Error Interrupt
  * @rmtoll CR          CEIE        LL_DMA2D_DisableIT_CE
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_DisableIT_CE(DMA2D_TypeDef *DMA2Dx)
{
  CLEAR_BIT(DMA2Dx->CR, DMA2D_CR_CEIE);
}

/**
  * @brief  Disable CLUT Transfer Complete Interrupt
  * @rmtoll CR          CTCIE        LL_DMA2D_DisableIT_CTC
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_DisableIT_CTC(DMA2D_TypeDef *DMA2Dx)
{
  CLEAR_BIT(DMA2Dx->CR, DMA2D_CR_CTCIE);
}

/**
  * @brief  Disable CLUT Access Error Interrupt
  * @rmtoll CR          CAEIE        LL_DMA2D_DisableIT_CAE
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_DisableIT_CAE(DMA2D_TypeDef *DMA2Dx)
{
  CLEAR_BIT(DMA2Dx->CR, DMA2D_CR_CAEIE);
}

/**
  * @brief  Disable Transfer Watermark Interrupt
  * @rmtoll CR          TWIE        LL_DMA2D_DisableIT_TW
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_DisableIT_TW(DMA2D_TypeDef *DMA2Dx)
{
  CLEAR_BIT(DMA2Dx->CR, DMA2D_CR_TWIE);
}

/**
  * @brief  Disable Transfer Complete Interrupt
  * @rmtoll CR          TCIE        LL_DMA2D_DisableIT_TC
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_DisableIT_TC(DMA2D_TypeDef *DMA2Dx)
{
  CLEAR_BIT(DMA2Dx->CR, DMA2D_CR_TCIE);
}

/**
  * @brief  Disable Transfer Error Interrupt
  * @rmtoll CR          TEIE        LL_DMA2D_DisableIT_TE
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
__STATIC_INLINE void LL_DMA2D_DisableIT_TE(DMA2D_TypeDef *DMA2Dx)
{
  CLEAR_BIT(DMA2Dx->CR, DMA2D_CR_TEIE);
}

/**
  * @brief  Check if the DMA2D Configuration Error interrupt source is enabled or disabled.
  * @rmtoll CR          CEIE        LL_DMA2D_IsEnabledIT_CE
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsEnabledIT_CE(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->CR, DMA2D_CR_CEIE) == (DMA2D_CR_CEIE));
}

/**
  * @brief  Check if the DMA2D CLUT Transfer Complete interrupt source is enabled or disabled.
  * @rmtoll CR          CTCIE        LL_DMA2D_IsEnabledIT_CTC
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsEnabledIT_CTC(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->CR, DMA2D_CR_CTCIE) == (DMA2D_CR_CTCIE));
}

/**
  * @brief  Check if the DMA2D CLUT Access Error interrupt source is enabled or disabled.
  * @rmtoll CR          CAEIE        LL_DMA2D_IsEnabledIT_CAE
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsEnabledIT_CAE(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->CR, DMA2D_CR_CAEIE) == (DMA2D_CR_CAEIE));
}

/**
  * @brief  Check if the DMA2D Transfer Watermark interrupt source is enabled or disabled.
  * @rmtoll CR          TWIE        LL_DMA2D_IsEnabledIT_TW
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsEnabledIT_TW(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->CR, DMA2D_CR_TWIE) == (DMA2D_CR_TWIE));
}

/**
  * @brief  Check if the DMA2D Transfer Complete interrupt source is enabled or disabled.
  * @rmtoll CR          TCIE        LL_DMA2D_IsEnabledIT_TC
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsEnabledIT_TC(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->CR, DMA2D_CR_TCIE) == (DMA2D_CR_TCIE));
}

/**
  * @brief  Check if the DMA2D Transfer Error interrupt source is enabled or disabled.
  * @rmtoll CR          TEIE        LL_DMA2D_IsEnabledIT_TE
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t LL_DMA2D_IsEnabledIT_TE(DMA2D_TypeDef *DMA2Dx)
{
  return (uint32_t)(READ_BIT(DMA2Dx->CR, DMA2D_CR_TEIE) == (DMA2D_CR_TEIE));
}



/**
  * @}
  */

#if defined(USE_FULL_LL_DRIVER)
/** @defgroup DMA2D_LL_EF_Init_Functions Initialization and De-initialization Functions
  * @{
  */

ErrorStatus LL_DMA2D_DeInit(DMA2D_TypeDef *DMA2Dx);
ErrorStatus LL_DMA2D_Init(DMA2D_TypeDef *DMA2Dx, LL_DMA2D_InitTypeDef *DMA2D_InitStruct);
void LL_DMA2D_StructInit(LL_DMA2D_InitTypeDef *DMA2D_InitStruct);
void LL_DMA2D_ConfigLayer(DMA2D_TypeDef *DMA2Dx, LL_DMA2D_LayerCfgTypeDef *DMA2D_LayerCfg, uint32_t LayerIdx);
void LL_DMA2D_LayerCfgStructInit(LL_DMA2D_LayerCfgTypeDef *DMA2D_LayerCfg);
void LL_DMA2D_ConfigOutputColor(DMA2D_TypeDef *DMA2Dx, LL_DMA2D_ColorTypeDef *DMA2D_ColorStruct);
uint32_t LL_DMA2D_GetOutputBlueColor(DMA2D_TypeDef *DMA2Dx, uint32_t ColorMode);
uint32_t LL_DMA2D_GetOutputGreenColor(DMA2D_TypeDef *DMA2Dx, uint32_t ColorMode);
uint32_t LL_DMA2D_GetOutputRedColor(DMA2D_TypeDef *DMA2Dx, uint32_t ColorMode);
uint32_t LL_DMA2D_GetOutputAlphaColor(DMA2D_TypeDef *DMA2Dx, uint32_t ColorMode);
void LL_DMA2D_ConfigSize(DMA2D_TypeDef *DMA2Dx, uint32_t NbrOfLines, uint32_t NbrOfPixelsPerLines);

/**
  * @}
  */
#endif /* USE_FULL_LL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* defined (DMA2D) */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* STM32L4xx_LL_DMA2D_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
