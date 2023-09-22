/* Copyright (c) 2010-2014, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 * Neither the name of the author nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "lpclib.h"
#include "bsp-fifisdr.h"

#include "usbuser.h"



/** Enumerate all string descriptors.
 *
 *  Use these names in the string descriptor table and the device/configuration descriptors
 *  to achieve an automatic numbering of all string descriptors.
 *
 *  The first entry must be assigned index 1.
 *  The last entry must be USB_NUM_STRING_DESCRIPTORS_PLUS1.
 */
enum {
    USBSTR_IDVENDOR = 1,
    USBSTR_IDPRODUCT,
    USBSTR_SERIALNUMBER,

    USBSTR_SOUNDCARD_IQ,
    USBSTR_STREAM1,
    USBSTR_PREAMP_UDA1361,
    USBSTR_BASEBAND,

    USBSTR_SOUNDCARD_DSP,
    USBSTR_DEMOD_AM_INPUT_TERMINAL,
    USBSTR_DEMOD_FM_INPUT_TERMINAL,
    USBSTR_DEMOD_LSB_INPUT_TERMINAL,
    USBSTR_DEMOD_USB_INPUT_TERMINAL,
    USBSTR_COMPRESSOR,
    USBSTR_DEMOD_OUTPUT_TERMINAL,

    USB_NUM_STRING_DESCRIPTORS_PLUS1
};


#define USB_NUM_LANGUAGES                   2

static DECLARE_LANGUAGE_ID_DESCRIPTOR(fifisdr_languages,
                                      USB_NUM_LANGUAGES,
                                      0x0407,   /* German           */
                                      0x0409,   /* English          */
                                      );

static DECLARE_STRING_DESCRIPTOR(str_idVendor_en, L"www.ov-lennestadt.de");
static DECLARE_STRING_DESCRIPTOR(str_idProduct_en, L"FiFi-SDR");

static DECLARE_STRING_DESCRIPTOR(str_soundcard_iq_en, L"FiFi-SDR Soundcard");
static DECLARE_STRING_DESCRIPTOR(str_stream1_de, L"UDA1361 Eingang");
static DECLARE_STRING_DESCRIPTOR(str_stream1_en, L"UDA1361 Input");
static DECLARE_STRING_DESCRIPTOR(str_preamp_uda_de, L"VV");
static DECLARE_STRING_DESCRIPTOR(str_preamp_uda_en, L"Preamp");
static DECLARE_STRING_DESCRIPTOR(str_baseband_de, L"Basisband (I/Q)");
static DECLARE_STRING_DESCRIPTOR(str_baseband_en, L"Complex Baseband (I/Q)");

static DECLARE_STRING_DESCRIPTOR(str_soundcard_dsp_de, L"FiFi-SDR AM/FM/SSB Empfänger");
static DECLARE_STRING_DESCRIPTOR(str_soundcard_dsp_en, L"FiFi-SDR AM/FM/SSB Receiver");
static DECLARE_STRING_DESCRIPTOR(str_demod_am_en, L"AM");
static DECLARE_STRING_DESCRIPTOR(str_demod_fm_en, L"FM");
static DECLARE_STRING_DESCRIPTOR(str_demod_lsb_en, L"LSB");
static DECLARE_STRING_DESCRIPTOR(str_demod_usb_en, L"USB");
static DECLARE_STRING_DESCRIPTOR(str_compressor_de, L"AGC");
static DECLARE_STRING_DESCRIPTOR(str_compressor_en, L"AGC");
static DECLARE_STRING_DESCRIPTOR(str_demodulator_mode_de, L"Modulationsart");
static DECLARE_STRING_DESCRIPTOR(str_demodulator_mode_en, L"Modulation");


/* The serial number is a special string in RAM that will be filled in at initialization time.
 * It holds a 64 character string with the serial number of the LPC device.
 * The string is initialized to a safe default.
 */
static __PACKED(struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    wchar_t     text[32];
}) str_iSerialNumber = {
    .bLength = 4,
    .bDescriptorType = USBD_STRING,
    .text = {L'*',},
};



static const struct {
    uint8_t id;
    const void *desc[USB_NUM_LANGUAGES];
} fifisdr_stringDescriptors[USB_NUM_STRING_DESCRIPTORS_PLUS1-1] =
{
    {   .id = USBSTR_IDVENDOR,
        .desc = {
            &str_idVendor_en,
            &str_idVendor_en,
        },
    },
    {   .id = USBSTR_IDPRODUCT,
        .desc = {
            &str_idProduct_en,
            &str_idProduct_en,
        },
    },
    {   .id = USBSTR_SERIALNUMBER,
        .desc = {
            &str_iSerialNumber,
            &str_iSerialNumber,
        },
    },

    {   .id = USBSTR_SOUNDCARD_IQ,
        .desc = {
            &str_soundcard_iq_en,
            &str_soundcard_iq_en,
        },
    },
    {   .id = USBSTR_STREAM1,
        .desc = {
            &str_stream1_de,
            &str_stream1_en,
        },
    },
    {   .id = USBSTR_PREAMP_UDA1361,
        .desc = {
            &str_preamp_uda_de,
            &str_preamp_uda_en,
        },
    },
    {   .id = USBSTR_BASEBAND,
        .desc = {
            &str_baseband_de,
            &str_baseband_en,
        },
    },

    {   .id = USBSTR_SOUNDCARD_DSP,
        .desc = {
            &str_soundcard_dsp_de,
            &str_soundcard_dsp_en,
        },
    },
    {   .id = USBSTR_DEMOD_AM_INPUT_TERMINAL,
        .desc = {
            &str_demod_am_en,
            &str_demod_am_en,
        },
    },
    {   .id = USBSTR_DEMOD_FM_INPUT_TERMINAL,
        .desc = {
            &str_demod_fm_en,
            &str_demod_fm_en,
        },
    },
    {   .id = USBSTR_DEMOD_LSB_INPUT_TERMINAL,
        .desc = {
            &str_demod_lsb_en,
            &str_demod_lsb_en,
        },
    },
    {   .id = USBSTR_DEMOD_USB_INPUT_TERMINAL,
        .desc = {
            &str_demod_usb_en,
            &str_demod_usb_en,
        },
    },
    {.id = USBSTR_COMPRESSOR,
        .desc = {
            &str_compressor_de,
            &str_compressor_en,
        },
    },
    {.id = USBSTR_DEMOD_OUTPUT_TERMINAL,
        .desc = {
            &str_demodulator_mode_de,
            &str_demodulator_mode_en,
        },
    },
};



/** Return pointer to string descriptor of given index and language.
 */
static LPCLIB_Result fifisdr_getString (uint8_t index,
                                        uint16_t languageId,
                                        const void **pString)
{
    uint16_t theLanguage;
    uint16_t i;


    /* Special handling for index zero. This is the descriptor which specifies the ID codes
     * of the supported languages.
     */
    if (index == 0) {
        *pString = (void *)&fifisdr_languages;

        return LPCLIB_SUCCESS;
    }

    /* Determine the language index in the string table. Fall back to index zero (first language)
     * if an unknown language ID is specified.
     */
    theLanguage = 0;
    for (i = 0; i < USB_NUM_LANGUAGES; i++) {
        if (fifisdr_languages.langId[i] == languageId) {
            theLanguage = i;
            break;
        }
    }

    /* Search the list */
    for (i = 0; i < USB_NUM_STRING_DESCRIPTORS_PLUS1 - 1; i++) {
        /* Does the specified index match the string ID? */
        if (fifisdr_stringDescriptors[i].id == index) {
            *pString = fifisdr_stringDescriptors[i].desc[theLanguage];

            return LPCLIB_SUCCESS;
        }
    }

    return LPCLIB_ILLEGAL_PARAMETER;
}



/** Descriptor structures of application-specific size. */
_USBAUDIO_ControlInterfaceHeaderDescriptorDef(USBAPP_IQControlInterfaceHeaderDescriptor, 1);    /* n = 1 streaming interface */
_USBAUDIO_FeatureUnitDescriptorDef(USBAPP_IQFeatureUnitDescriptor, 2, uint8_t);                 /* ch = 2 */
_USBAUDIO_ControlInterfaceHeaderDescriptorDef(USBAPP_DspControlInterfaceHeaderDescriptor, 1);   /* n = 1 streaming interface */
_USBAUDIO_ProcessingUnitDescriptorDef(USBAPP_AgcDescriptor, 1, 1);                              /* p = 1 channel, n = 1 byte for controls */
_USBAUDIO_FeatureUnitDescriptorDef(USBAPP_DspFeatureUnitDescriptor, 1, uint8_t);                /* ch = 1 */
_USBAUDIO_SelectorUnitDescriptorDef(USBAPP_DspModulationSelector, 4);                           /* 4 inputs */
_USBAUDIO_TypeIFormatDescriptorDef(USBAPP_FormatDescriptor1, 1);                                /* n = 1 sample frequency */
_USBAUDIO_TypeIFormatDescriptorDef(USBAPP_FormatDescriptor2, 2);                                /* n = 2 sample frequencies */
_USBAUDIO_TypeIFormatDescriptorDef(USBAPP_FormatDescriptor3, 3);                                /* n = 3 sample frequencies */



/** FiFi-SDR device descriptor. */
static const USB_DeviceDescriptor fifisdr_deviceDescriptor = {
    .bLength                    = USBD_SIZE_DEVICE,
    .bDescriptorType            = USBD_DEVICE,
    .bcdUSB                     = 0x0110,
    .bDeviceClass               = USBC_BASE,
    .bDeviceSubClass            = 0,
    .bDeviceProtocol            = 0,
    .bMaxPacketSize0            = 64,
    .idVendor                   = 0x16C0,
    .idProduct                  = 0x05DC,
    .bcdDevice                  = 0x0101,               /* 0x0100 First version ("FiFi-SDR DSP")
                                                         * 0x0101 Add selector and processing units
                                                         */
    .iManufacturer              = USBSTR_IDVENDOR,
    .iProduct                   = USBSTR_IDPRODUCT,
    .iSerialNumber              = USBSTR_SERIALNUMBER,
    .bNumConfigurations         = USBCONFIG_NUM_CONFIGURATIONS,
};


/** FiFi-SDR configuration 1 (96 kHz) */
static const __PACKED(struct {
    USB_ConfigurationDescriptor                     config;
    USB_InterfaceDescriptor                         softrockInterface;

#if USBCONFIG_SOUNDCARD
    struct {
        USB_InterfaceDescriptor                     interface;
        struct _iqControlInterfaceSpec96 {
            USBAPP_IQControlInterfaceHeaderDescriptor   interfaceHeader;
            USBAUDIO_InputTerminalDescriptor            inputTerminal;
            USBAPP_IQFeatureUnitDescriptor              featureUnit;
            USBAUDIO_OutputTerminalDescriptor           outputTerminal;
        } specification;
    } iqControlInterface;
    struct {
        USB_InterfaceDescriptor                     interface;
    } iqStreamingInterface_alt0;
    struct {
        USB_InterfaceDescriptor                     interface;
        USBAUDIO_StreamingInterfaceDescriptor       streamingInterface;
        USBAPP_FormatDescriptor2                    formatType;
        USB_EndpointDescriptorIso                   endpoint;
        USBAUDIO_IsoDataEndpointDescriptor          dataEndpoint;
    } iqStreamingInterface_alt1;
    struct {
        USB_InterfaceDescriptor                     interface;
        USBAUDIO_StreamingInterfaceDescriptor       streamingInterface;
        USBAPP_FormatDescriptor2                    formatType;
        USB_EndpointDescriptorIso                   endpoint;
        USBAUDIO_IsoDataEndpointDescriptor          dataEndpoint;
    } iqStreamingInterface_alt2;

  #if USBCONFIG_DSP
    struct {
        USB_InterfaceDescriptor                     interface;
        struct _dspControlInterfaceSpec96 {
            USBAPP_DspControlInterfaceHeaderDescriptor  interfaceHeader;
            USBAUDIO_InputTerminalDescriptor            inputTerminalAM;
            USBAPP_DspFeatureUnitDescriptor             featureUnitAM;
            USBAUDIO_InputTerminalDescriptor            inputTerminalFM;
            USBAPP_DspFeatureUnitDescriptor             featureUnitFM;
            USBAUDIO_InputTerminalDescriptor            inputTerminalLSB;
            USBAPP_DspFeatureUnitDescriptor             featureUnitLSB;
            USBAUDIO_InputTerminalDescriptor            inputTerminalUSB;
            USBAPP_DspFeatureUnitDescriptor             featureUnitUSB;
            USBAPP_DspModulationSelector                selectorUnit;
            USBAPP_AgcDescriptor                        agcUnit;
            USBAUDIO_OutputTerminalDescriptor           outputTerminal;
        } specification;
    } dspControlInterface;
    struct {
        USB_InterfaceDescriptor                     interface;
    } dspStreamingInterface_alt0;
    struct {
        USB_InterfaceDescriptor                     interface;
        USBAUDIO_StreamingInterfaceDescriptor       streamingInterface;
        USBAPP_FormatDescriptor1                    formatType;
        USB_EndpointDescriptorIso                   endpoint;
        USBAUDIO_IsoDataEndpointDescriptor          dataEndpoint;
    } dspStreamingInterface_alt1;
  #endif
#endif  /* #if USBCONFIG_SOUNDCARD */
}) fifisdr_configuration1_96k = {

    .config = {
        .bLength                = USBD_SIZE_CONFIGURATION,
        .bDescriptorType        = USBD_CONFIGURATION,
        .wTotalLength           = sizeof(fifisdr_configuration1_96k),
        .bNumInterfaces         = USBCONFIG_NUM_INTERFACES,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = 0x80,     /* bus-powered */
        .bMaxPower              = 250/2,
    },

    /* Softrock-40 Interface */
    .softrockInterface = {
        .bLength                = USBD_SIZE_INTERFACE,
        .bDescriptorType        = USBD_INTERFACE,
        .bInterfaceNumber       = USBCONFIG_INTERFACE_SOFTROCK_1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 0,
        .bInterfaceClass        = USBC_VENDOR_SPECIFIC,
        .bInterfaceSubClass     = 0,
        .bInterfaceProtocol     = 0,
        .iInterface             = 0,
    },

#if USBCONFIG_SOUNDCARD
    .iqControlInterface = {
        /* Standard AudioControl Interface */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_CONTROL_1,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 0,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOCONTROL,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_SOUNDCARD_IQ,
        },

        .specification = {
            /* Class-Specific Audio Control Interface */
            .interfaceHeader = {
                .bLength                = USBD_SIZE_AUDIO_ST_HEADER(1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_HEADER,
                .bcdADC                 = 0x0100,   /* 1.0 */
                .wTotalLength           = sizeof(struct _iqControlInterfaceSpec96),
                .bInCollection          = 1,
                .baInterfaceNr          = {
                    USBCONFIG_INTERFACE_AUDIO_STREAMING_1,
                },
            },

            /* Input Terminal (I/Q) */
            .inputTerminal = {
                .bLength                = USBD_SIZE_AUDIO_ST_INPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN_IQ,
                .wTerminalType          = USBAUDIO_TERMINAL_INPUT_MICROPHONE,//USBAUDIO_TERMINAL_EXTERNAL_LINE_CONNECTOR,
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 2,
                .wChannelConfig         = 0x0003,
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_BASEBAND,
            },

            /* Feature Unit (UDA) */
            .featureUnit = {
                .bLength                = USBD_SIZE_AUDIO_ST_FEATURE_UNIT(2,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_FEATURE_UNIT,
                .bUnitID                = USBCONFIG_UNIT_FEATURE_IQ,
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN_IQ,
                .bControlSize           = 1,        /* n = 1 Byte/channel */
                .bmaControls            = {
                    0x02,               /* bmaControls(0)       Master: Volume */
                    0x00,               /* bmaControls(1)                      */
                    0x00,               /* bmaControls(2)                      */
                },
                .iFeature               = USBSTR_PREAMP_UDA1361,
            },

            /* Output Terminal I/Q */
            .outputTerminal = {
                .bLength                = USBD_SIZE_AUDIO_ST_OUTPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_OUTPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_OUT_IQ,
                .wTerminalType          = USBAUDIO_TERMINAL_USB_STREAMING,
                .bAssocTerminal         = 0,        /* no association */
                .bSourceID              = USBCONFIG_UNIT_FEATURE_IQ,
                .iTerminal              = 0,//USBSTR_BASEBAND,
            },
        },
    },

    .iqStreamingInterface_alt0 = {
        /* Standard AudioStreaming Interface 1 (alternate setting #0) */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_STREAMING_1,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 0,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_STREAM1,
        },
    },

    .iqStreamingInterface_alt1 = {
        /* Standard AudioStreaming Interface 1 (alternate setting #1) */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_STREAMING_1,
            .bAlternateSetting      = 1,
            .bNumEndpoints          = 1,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_STREAM1,
        },

        /* Audio Stream Audio Class */
        .streamingInterface = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 1,        /* AS_GENERAL */
            .bTerminalLink          = USBCONFIG_UNIT_TERMINAL_OUT_IQ,
            .bDelay                 = 1,
            .wFormatTag             = USBAUDIO_FORMATTAG_PCM,
        },

        /* Format Type Audio */
        .formatType = {
            .bLength                = 14,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 2,        /* FORMAT_TYPE */
            .bFormatType            = USBAUDIO_FORMAT_TYPE_I,
            .bNrChannels            = 2,
            .bSubFrameSize          = 4,
            .bBitResolution         = 32,
            .bSamFreqType           = 2,        /* 2 frequencies follow */
            .tSamFreq               = {
                { LE24(48000UL), },
                { LE24(96000UL), },
            },
        },

        /* Audio streaming IN endpoint */
        .endpoint = {
            .bLength                = USBD_SIZE_ENDPOINT_ISO,
            .bDescriptorType        = USBD_ENDPOINT,
            .bEndpointAddress       = USBCONFIG_STREAMING_EP1,
            .bmAttributes           = 0x05,     /* isochronous, async, data */
            .wMaxPacketSize         = USBCONFIG_ISOC_SIZE1_32,
            .bInterval              = 1,
            .bRefresh               = 0,
            .bSyncAddress           = 0,
        },

        /* Class specific audio endpoint */
        .dataEndpoint = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_ENDPOINT,
            .bDescriptorSubtype     = 1,        /* EP_GENERAL */
            .bmAttributes           = 1,        /* Sample rate control */
            .bLockDelayUnits        = 0,
            .wLockDelay             = 0,
        },
    },

    .iqStreamingInterface_alt2 = {
        /* Standard AudioStreaming Interface 1 (alternate setting #2) */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_STREAMING_1,
            .bAlternateSetting      = 2,
            .bNumEndpoints          = 1,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_STREAM1,
        },

        /* Audio Stream Audio Class */
        .streamingInterface = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 1,        /* AS_GENERAL */
            .bTerminalLink          = USBCONFIG_UNIT_TERMINAL_OUT_IQ,
            .bDelay                 = 1,
            .wFormatTag             = USBAUDIO_FORMATTAG_PCM,
        },

        /* Format Type Audio */
        .formatType = {
            .bLength                = 14,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 2,        /* FORMAT_TYPE */
            .bFormatType            = USBAUDIO_FORMAT_TYPE_I,
            .bNrChannels            = 2,
            .bSubFrameSize          = 2,
            .bBitResolution         = 16,
            .bSamFreqType           = 2,        /* 2 frequencies follow */
            .tSamFreq               = {
                { LE24(48000UL), },
                { LE24(96000UL), },
            },
        },

        /* Audio streaming IN endpoint */
        .endpoint = {
            .bLength                = USBD_SIZE_ENDPOINT_ISO,
            .bDescriptorType        = USBD_ENDPOINT,
            .bEndpointAddress       = USBCONFIG_STREAMING_EP1,
            .bmAttributes           = 0x05,     /* isochronous, async, data */
            .wMaxPacketSize         = USBCONFIG_ISOC_SIZE1_16,
            .bInterval              = 1,
            .bRefresh               = 0,
            .bSyncAddress           = 0,
        },

        /* Class specific audio endpoint */
        .dataEndpoint = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_ENDPOINT,
            .bDescriptorSubtype     = 1,        /* EP_GENERAL */
            .bmAttributes           = 1,        /* Sample rate control */
            .bLockDelayUnits        = 0,
            .wLockDelay             = 0,
        },
    },

  #if USBCONFIG_DSP
    .dspControlInterface = {
        /* Standard AudioControl Interface */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_DSP_CONTROL,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 0,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOCONTROL,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_SOUNDCARD_DSP,
        },

        .specification = {
            /* Class-Specific Audio Control Interface */
            .interfaceHeader = {
                .bLength                = USBD_SIZE_AUDIO_ST_HEADER(1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_HEADER,
                .bcdADC                 = 0x0100,   /* 1.0 */
                .wTotalLength           = sizeof(struct _dspControlInterfaceSpec96),
                .bInCollection          = 1,
                .baInterfaceNr          = {
                    USBCONFIG_INTERFACE_AUDIO_DSP_STREAMING,
                },
            },

            /* Input Terminal (AM) */
            .inputTerminalAM = {
                .bLength                = USBD_SIZE_AUDIO_ST_INPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN_DSP1,
                .wTerminalType          = USBAUDIO_TERMINAL_EXTERNAL_DIGITAL_AUDIO_INTERFACE,
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 1,
                .wChannelConfig         = 0x0004,   /* Center C */
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_DEMOD_AM_INPUT_TERMINAL,
            },

            /* Feature Unit (AM) */
            .featureUnitAM = {
                .bLength                = USBD_SIZE_AUDIO_ST_FEATURE_UNIT(1,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_FEATURE_UNIT,
                .bUnitID                = USBCONFIG_UNIT_FEATURE_DSP_AM,
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN_DSP1,
                .bControlSize           = 1,        /* n = 1 Byte/channel */
                .bmaControls            = {
                    0x02,               /* bmaControls(0)       Master: Volume */
                    0x00,               /* bmaControls(1)                      */
                },
                .iFeature               = 0,
            },

            /* Input Terminal (FM) */
            .inputTerminalFM = {
                .bLength                = USBD_SIZE_AUDIO_ST_INPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN_DSP2,
                .wTerminalType          = USBAUDIO_TERMINAL_INPUT_UNDEFINED,    /* Invisible in Windows! */
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 1,
                .wChannelConfig         = 0x0004,   /* Center C */
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_DEMOD_FM_INPUT_TERMINAL,
            },

            /* Feature Unit (FM) */
            .featureUnitFM = {
                .bLength                = USBD_SIZE_AUDIO_ST_FEATURE_UNIT(1,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_FEATURE_UNIT,
                .bUnitID                = USBCONFIG_UNIT_FEATURE_DSP_FM,
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN_DSP2,
                .bControlSize           = 1,        /* n = 1 Byte/channel */
                .bmaControls            = {
                    0x02,               /* bmaControls(0)       Master: Volume */
                    0x00,               /* bmaControls(1)                      */
                },
                .iFeature               = 0,
            },

            /* Input Terminal (LSB) */
            .inputTerminalLSB = {
                .bLength                = USBD_SIZE_AUDIO_ST_INPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN_DSP3,
                .wTerminalType          = USBAUDIO_TERMINAL_INPUT_UNDEFINED,    /* Invisible in Windows! */
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 1,
                .wChannelConfig         = 0x0004,   /* Center C */
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_DEMOD_LSB_INPUT_TERMINAL,
            },

            /* Feature Unit (LSB) */
            .featureUnitLSB = {
                .bLength                = USBD_SIZE_AUDIO_ST_FEATURE_UNIT(1,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_FEATURE_UNIT,
                .bUnitID                = USBCONFIG_UNIT_FEATURE_DSP_LSB,
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN_DSP3,
                .bControlSize           = 1,        /* n = 1 Byte/channel */
                .bmaControls            = {
                    0x02,               /* bmaControls(0)       Master: Volume */
                    0x00,               /* bmaControls(1)                      */
                },
                .iFeature               = 0,
            },

            /* Input Terminal (USB) */
            .inputTerminalUSB = {
                .bLength                = USBD_SIZE_AUDIO_ST_INPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN_DSP4,
                .wTerminalType          = USBAUDIO_TERMINAL_INPUT_UNDEFINED,    /* Invisible in Windows! */
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 1,
                .wChannelConfig         = 0x0004,   /* Center C */
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_DEMOD_USB_INPUT_TERMINAL,
            },

            /* Feature Unit (USB) */
            .featureUnitUSB = {
                .bLength                = USBD_SIZE_AUDIO_ST_FEATURE_UNIT(1,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_FEATURE_UNIT,
                .bUnitID                = USBCONFIG_UNIT_FEATURE_DSP_USB,
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN_DSP4,
                .bControlSize           = 1,        /* n = 1 Byte/channel */
                .bmaControls            = {
                    0x02,               /* bmaControls(0)       Master: Volume */
                    0x00,               /* bmaControls(1)                      */
                },
                .iFeature               = 0,
            },

            .selectorUnit = {
                .bLength                = USBD_SIZE_AUDIO_ST_SELECTOR_UNIT(4),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_SELECTOR_UNIT,
                .bUnitID                = USBCONFIG_UNIT_SELECTOR_DSP,
                .bNrInPins              = 4,
                .baSourceID             = {
                    USBCONFIG_UNIT_FEATURE_DSP_AM,
                    USBCONFIG_UNIT_FEATURE_DSP_FM,
                    USBCONFIG_UNIT_FEATURE_DSP_LSB,
                    USBCONFIG_UNIT_FEATURE_DSP_USB,
                },
                .iSelector              = 0,
            },

            /* Processing Unit (AGC for DSP) */
            .agcUnit = {
                .bLength                = USBD_SIZE_AUDIO_ST_PROCESSING_UNIT(1,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_PROCESSING_UNIT,
                .bUnitID                = USBCONFIG_UNIT_PROCESSING_DSP,
                .wProcessType           = USBAC_PU_DYN_RANGE_COMP_PROCESS,
                .bNrInPins              = 1,
                .baSourceID             = {USBCONFIG_UNIT_SELECTOR_DSP,},
                .bNrChannels            = 1,
                .wChannelConfig         = 0x0004,   /* D2: Center Front (C) */
                .iChannelNames          = 0,
                .bControlSize           = 1,
            #if 0
                .bmControls             = {0x3F,},  /* D0: Enable Processing
                                                     * D1: Compression Ratio
                                                     * D2: Max Amplitude
                                                     * D3: Threshold
                                                     * D4: Attack Time
                                                     * D5: Release Time
                                                     */
            #else
                .bmControls             = {0x30,},  /* D4: Attack Time
                                                     * D5: Release Time
                                                     */
            #endif
                .iProcessing            = USBSTR_COMPRESSOR,
            },

            /* Output Terminal DSP */
            .outputTerminal = {
                .bLength                = USBD_SIZE_AUDIO_ST_OUTPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_OUTPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_OUT_DSP,
                .wTerminalType          = USBAUDIO_TERMINAL_USB_STREAMING,
                .bAssocTerminal         = 0,        /* no association */
                .bSourceID              = USBCONFIG_UNIT_PROCESSING_DSP,
                .iTerminal              = USBSTR_DEMOD_OUTPUT_TERMINAL,
            },
        },
    },

    .dspStreamingInterface_alt0 = {
        /* Standard AudioStreaming Interface 2 (alternate setting #0) */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_DSP_STREAMING,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 0,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = 0,
        },
    },

    .dspStreamingInterface_alt1 = {
        /* Standard AudioStreaming Interface 2 (alternate setting #1) */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_DSP_STREAMING,
            .bAlternateSetting      = 1,
            .bNumEndpoints          = 1,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = 0,
        },

        /* Audio Stream Audio Class */
        .streamingInterface = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 1,        /* AS_GENERAL */
            .bTerminalLink          = USBCONFIG_UNIT_TERMINAL_OUT_DSP,
            .bDelay                 = 1,
            .wFormatTag             = USBAUDIO_FORMATTAG_PCM,
        },

        /* Format Type Audio */
        .formatType = {
            .bLength                = 11,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 2,        /* FORMAT_TYPE */
            .bFormatType            = USBAUDIO_FORMAT_TYPE_I,
            .bNrChannels            = 1,
            .bSubFrameSize          = 2,
            .bBitResolution         = 16,
            .bSamFreqType           = 1,        /* 1 frequency follows */
            .tSamFreq               = {
                { LE24(48000ul), },
            },
        },

        /* Audio streaming IN endpoint */
        .endpoint = {
            .bLength                = USBD_SIZE_ENDPOINT_ISO,
            .bDescriptorType        = USBD_ENDPOINT,
            .bEndpointAddress       = USBCONFIG_STREAMING_EP2,
            .bmAttributes           = 0x05,     /* isochronous, async, data */
            .wMaxPacketSize         = USBCONFIG_ISOC_SIZE2,
            .bInterval              = 1,
            .bRefresh               = 0,
            .bSyncAddress           = 0,
        },

        /* Class specific audio endpoint */
        .dataEndpoint = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_ENDPOINT,
            .bDescriptorSubtype     = 1,        /* EP_GENERAL */
            .bmAttributes           = 1,        /* Sample rate control */
            .bLockDelayUnits        = 0,
            .wLockDelay             = 0,
        },
    },
  #endif
#endif  /* #if USBCONFIG_SOUNDCARD */
};


/** FiFi-SDR configuration 1 (192 kHz) */
static const __PACKED(struct {
    USB_ConfigurationDescriptor                     config;
    USB_InterfaceDescriptor                         softrockInterface;

#if USBCONFIG_SOUNDCARD
    struct {
        USB_InterfaceDescriptor                     interface;
        struct _iqControlInterfaceSpec192 {
            USBAPP_IQControlInterfaceHeaderDescriptor   interfaceHeader;
            USBAUDIO_InputTerminalDescriptor            inputTerminal;
            USBAUDIO_OutputTerminalDescriptor           outputTerminal;
        } specification;
    } iqControlInterface;
    struct {
        USB_InterfaceDescriptor                     interface;
    } iqStreamingInterface_alt0;
    struct {
        USB_InterfaceDescriptor                     interface;
        USBAUDIO_StreamingInterfaceDescriptor       streamingInterface;
        USBAPP_FormatDescriptor2                    formatType;
        USB_EndpointDescriptorIso                   endpoint;
        USBAUDIO_IsoDataEndpointDescriptor          dataEndpoint;
    } iqStreamingInterface_alt1;
    struct {
        USB_InterfaceDescriptor                     interface;
        USBAUDIO_StreamingInterfaceDescriptor       streamingInterface;
        USBAPP_FormatDescriptor3                    formatType;
        USB_EndpointDescriptorIso                   endpoint;
        USBAUDIO_IsoDataEndpointDescriptor          dataEndpoint;
    } iqStreamingInterface_alt2;

  #if USBCONFIG_DSP
    struct {
        USB_InterfaceDescriptor                     interface;
        struct _dspControlInterfaceSpec192 {
            USBAPP_DspControlInterfaceHeaderDescriptor  interfaceHeader;
            USBAUDIO_InputTerminalDescriptor            inputTerminalAM;
            USBAPP_DspFeatureUnitDescriptor             featureUnitAM;
            USBAUDIO_InputTerminalDescriptor            inputTerminalFM;
            USBAPP_DspFeatureUnitDescriptor             featureUnitFM;
            USBAUDIO_InputTerminalDescriptor            inputTerminalLSB;
            USBAPP_DspFeatureUnitDescriptor             featureUnitLSB;
            USBAUDIO_InputTerminalDescriptor            inputTerminalUSB;
            USBAPP_DspFeatureUnitDescriptor             featureUnitUSB;
            USBAPP_DspModulationSelector                selectorUnit;
            USBAPP_AgcDescriptor                        agcUnit;
            USBAUDIO_OutputTerminalDescriptor           outputTerminal;
        } specification;
    } dspControlInterface;
    struct {
        USB_InterfaceDescriptor                     interface;
    } dspStreamingInterface_alt0;
    struct {
        USB_InterfaceDescriptor                     interface;
        USBAUDIO_StreamingInterfaceDescriptor       streamingInterface;
        USBAPP_FormatDescriptor1                    formatType;
        USB_EndpointDescriptorIso                   endpoint;
        USBAUDIO_IsoDataEndpointDescriptor          dataEndpoint;
    } dspStreamingInterface_alt1;
  #endif
#endif  /* #if USBCONFIG_SOUNDCARD */
}) fifisdr_configuration1_192k = {

    .config = {
        .bLength                = USBD_SIZE_CONFIGURATION,
        .bDescriptorType        = USBD_CONFIGURATION,
        .wTotalLength           = sizeof(fifisdr_configuration1_192k),
        .bNumInterfaces         = USBCONFIG_NUM_INTERFACES,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = 0x80,     /* bus-powered */
        .bMaxPower              = 400/2,
    },

    /* Softrock-40 Interface */
    .softrockInterface = {
        .bLength                = USBD_SIZE_INTERFACE,
        .bDescriptorType        = USBD_INTERFACE,
        .bInterfaceNumber       = USBCONFIG_INTERFACE_SOFTROCK_1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 0,
        .bInterfaceClass        = USBC_VENDOR_SPECIFIC,
        .bInterfaceSubClass     = 0,
        .bInterfaceProtocol     = 0,
        .iInterface             = 0,
    },

#if USBCONFIG_SOUNDCARD
    .iqControlInterface = {
        /* Standard AudioControl Interface */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_CONTROL_1,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 0,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOCONTROL,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_SOUNDCARD_IQ,
        },

        .specification = {
            /* Class-Specific Audio Control Interface */
            .interfaceHeader = {
                .bLength                = USBD_SIZE_AUDIO_ST_HEADER(1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_HEADER,
                .bcdADC                 = 0x0100,   /* 1.0 */
                .wTotalLength           = sizeof(struct _iqControlInterfaceSpec192),
                .bInCollection          = 1,
                .baInterfaceNr          = {
                    USBCONFIG_INTERFACE_AUDIO_STREAMING_1,
                },
            },

            /* Input Terminal (I/Q) */
            .inputTerminal = {
                .bLength                = USBD_SIZE_AUDIO_ST_INPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN_IQ,
                .wTerminalType          = USBAUDIO_TERMINAL_INPUT_MICROPHONE,//USBAUDIO_TERMINAL_EXTERNAL_LINE_CONNECTOR,
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 2,
                .wChannelConfig         = 0x0003,
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_BASEBAND,
            },

            /* NOTE: No feature unit in the 192 kHz version (no gain control possible) */

            /* Output Terminal I/Q */
            .outputTerminal = {
                .bLength                = USBD_SIZE_AUDIO_ST_OUTPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_OUTPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_OUT_IQ,
                .wTerminalType          = USBAUDIO_TERMINAL_USB_STREAMING,
                .bAssocTerminal         = 0,        /* no association */
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN_IQ,
                .iTerminal              = 0,//USBSTR_BASEBAND,
            },
        },
    },

    .iqStreamingInterface_alt0 = {
        /* Standard AudioStreaming Interface 1 (alternate setting #0) */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_STREAMING_1,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 0,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_STREAM1,
        },
    },

    .iqStreamingInterface_alt1 = {
        /* Standard AudioStreaming Interface 1 (alternate setting #1) */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_STREAMING_1,
            .bAlternateSetting      = 1,
            .bNumEndpoints          = 1,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_STREAM1,
        },

        /* Audio Stream Audio Class */
        .streamingInterface = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 1,        /* AS_GENERAL */
            .bTerminalLink          = USBCONFIG_UNIT_TERMINAL_OUT_IQ,
            .bDelay                 = 1,
            .wFormatTag             = USBAUDIO_FORMATTAG_PCM,
        },

        /* Format Type Audio */
        .formatType = {
            .bLength                = 14,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 2,        /* FORMAT_TYPE */
            .bFormatType            = USBAUDIO_FORMAT_TYPE_I,
            .bNrChannels            = 2,
            .bSubFrameSize          = 4,
            .bBitResolution         = 32,
            .bSamFreqType           = 2,        /* 2 frequencies follow */
            .tSamFreq               = {
                { LE24(48000UL), },
                { LE24(96000UL), },
            },
        },

        /* Audio streaming IN endpoint */
        .endpoint = {
            .bLength                = USBD_SIZE_ENDPOINT_ISO,
            .bDescriptorType        = USBD_ENDPOINT,
            .bEndpointAddress       = USBCONFIG_STREAMING_EP1,
            .bmAttributes           = 0x05,     /* isochronous, async, data */
            .wMaxPacketSize         = USBCONFIG_ISOC_SIZE1_32,
            .bInterval              = 1,
            .bRefresh               = 0,
            .bSyncAddress           = 0,
        },

        /* Class specific audio endpoint */
        .dataEndpoint = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_ENDPOINT,
            .bDescriptorSubtype     = 1,        /* EP_GENERAL */
            .bmAttributes           = 1,        /* Sample rate control */
            .bLockDelayUnits        = 0,
            .wLockDelay             = 0,
        },
    },

    .iqStreamingInterface_alt2 = {
        /* Standard AudioStreaming Interface 1 (alternate setting #2) */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_STREAMING_1,
            .bAlternateSetting      = 2,
            .bNumEndpoints          = 1,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_STREAM1,
        },

        /* Audio Stream Audio Class */
        .streamingInterface = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 1,        /* AS_GENERAL */
            .bTerminalLink          = USBCONFIG_UNIT_TERMINAL_OUT_IQ,
            .bDelay                 = 1,
            .wFormatTag             = USBAUDIO_FORMATTAG_PCM,
        },

        /* Format Type Audio */
        .formatType = {
            .bLength                = 17,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 2,        /* FORMAT_TYPE */
            .bFormatType            = USBAUDIO_FORMAT_TYPE_I,
            .bNrChannels            = 2,
            .bSubFrameSize          = 2,
            .bBitResolution         = 16,
            .bSamFreqType           = 3,        /* 3 frequencies follow */
            .tSamFreq               = {
                { LE24(48000UL), },
                { LE24(96000UL), },
                { LE24(192000UL), },
            },
        },

        /* Audio streaming IN endpoint */
        .endpoint = {
            .bLength                = USBD_SIZE_ENDPOINT_ISO,
            .bDescriptorType        = USBD_ENDPOINT,
            .bEndpointAddress       = USBCONFIG_STREAMING_EP1,
            .bmAttributes           = 0x05,     /* isochronous, async, data */
            .wMaxPacketSize         = USBCONFIG_ISOC_SIZE1_16,
            .bInterval              = 1,
            .bRefresh               = 0,
            .bSyncAddress           = 0,
        },

        /* Class specific audio endpoint */
        .dataEndpoint = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_ENDPOINT,
            .bDescriptorSubtype     = 1,        /* EP_GENERAL */
            .bmAttributes           = 1,        /* Sample rate control */
            .bLockDelayUnits        = 0,
            .wLockDelay             = 0,
        },
    },

  #if USBCONFIG_DSP
    .dspControlInterface = {
        /* Standard AudioControl Interface */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_DSP_CONTROL,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 0,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOCONTROL,
            .bInterfaceProtocol     = 0,
            .iInterface             = USBSTR_SOUNDCARD_DSP,
        },

        .specification = {
            /* Class-Specific Audio Control Interface */
            .interfaceHeader = {
                .bLength                = USBD_SIZE_AUDIO_ST_HEADER(1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_HEADER,
                .bcdADC                 = 0x0100,   /* 1.0 */
                .wTotalLength           = sizeof(struct _dspControlInterfaceSpec192),
                .bInCollection          = 1,
                .baInterfaceNr          = {
                    USBCONFIG_INTERFACE_AUDIO_DSP_STREAMING,
                },
            },

            /* Input Terminal (AM) */
            .inputTerminalAM = {
                .bLength                = USBD_SIZE_AUDIO_ST_INPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN_DSP1,
                .wTerminalType          = USBAUDIO_TERMINAL_EXTERNAL_DIGITAL_AUDIO_INTERFACE,
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 1,
                .wChannelConfig         = 0x0004,   /* Center C */
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_DEMOD_AM_INPUT_TERMINAL,
            },

            /* Feature Unit (AM) */
            .featureUnitAM = {
                .bLength                = USBD_SIZE_AUDIO_ST_FEATURE_UNIT(1,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_FEATURE_UNIT,
                .bUnitID                = USBCONFIG_UNIT_FEATURE_DSP_AM,
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN_DSP1,
                .bControlSize           = 1,        /* n = 1 Byte/channel */
                .bmaControls            = {
                    0x02,               /* bmaControls(0)       Master: Volume */
                    0x00,               /* bmaControls(1)                      */
                },
                .iFeature               = 0,
            },

            /* Input Terminal (FM) */
            .inputTerminalFM = {
                .bLength                = USBD_SIZE_AUDIO_ST_INPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN_DSP2,
                .wTerminalType          = USBAUDIO_TERMINAL_INPUT_UNDEFINED,    /* Invisible in Windows! */
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 1,
                .wChannelConfig         = 0x0004,   /* Center C */
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_DEMOD_FM_INPUT_TERMINAL,
            },

            /* Feature Unit (FM) */
            .featureUnitFM = {
                .bLength                = USBD_SIZE_AUDIO_ST_FEATURE_UNIT(1,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_FEATURE_UNIT,
                .bUnitID                = USBCONFIG_UNIT_FEATURE_DSP_FM,
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN_DSP2,
                .bControlSize           = 1,        /* n = 1 Byte/channel */
                .bmaControls            = {
                    0x02,               /* bmaControls(0)       Master: Volume */
                    0x00,               /* bmaControls(1)                      */
                },
                .iFeature               = 0,
            },

            /* Input Terminal (LSB) */
            .inputTerminalLSB = {
                .bLength                = USBD_SIZE_AUDIO_ST_INPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN_DSP3,
                .wTerminalType          = USBAUDIO_TERMINAL_INPUT_UNDEFINED,    /* Invisible in Windows! */
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 1,
                .wChannelConfig         = 0x0004,   /* Center C */
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_DEMOD_LSB_INPUT_TERMINAL,
            },

            /* Feature Unit (LSB) */
            .featureUnitLSB = {
                .bLength                = USBD_SIZE_AUDIO_ST_FEATURE_UNIT(1,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_FEATURE_UNIT,
                .bUnitID                = USBCONFIG_UNIT_FEATURE_DSP_LSB,
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN_DSP3,
                .bControlSize           = 1,        /* n = 1 Byte/channel */
                .bmaControls            = {
                    0x02,               /* bmaControls(0)       Master: Volume */
                    0x00,               /* bmaControls(1)                      */
                },
                .iFeature               = 0,
            },

            /* Input Terminal (USB) */
            .inputTerminalUSB = {
                .bLength                = USBD_SIZE_AUDIO_ST_INPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_INPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_IN_DSP4,
                .wTerminalType          = USBAUDIO_TERMINAL_INPUT_UNDEFINED,    /* Invisible in Windows! */
                .bAssocTerminal         = 0,        /* no association */
                .bNrChannels            = 1,
                .wChannelConfig         = 0x0004,   /* Center C */
                .iChannelNames          = 0,
                .iTerminal              = USBSTR_DEMOD_USB_INPUT_TERMINAL,
            },

            /* Feature Unit (USB) */
            .featureUnitUSB = {
                .bLength                = USBD_SIZE_AUDIO_ST_FEATURE_UNIT(1,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_FEATURE_UNIT,
                .bUnitID                = USBCONFIG_UNIT_FEATURE_DSP_USB,
                .bSourceID              = USBCONFIG_UNIT_TERMINAL_IN_DSP4,
                .bControlSize           = 1,        /* n = 1 Byte/channel */
                .bmaControls            = {
                    0x02,               /* bmaControls(0)       Master: Volume */
                    0x00,               /* bmaControls(1)                      */
                },
                .iFeature               = 0,
            },

            .selectorUnit = {
                .bLength                = USBD_SIZE_AUDIO_ST_SELECTOR_UNIT(4),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_SELECTOR_UNIT,
                .bUnitID                = USBCONFIG_UNIT_SELECTOR_DSP,
                .bNrInPins              = 4,
                .baSourceID             = {
                    USBCONFIG_UNIT_FEATURE_DSP_AM,
                    USBCONFIG_UNIT_FEATURE_DSP_FM,
                    USBCONFIG_UNIT_FEATURE_DSP_LSB,
                    USBCONFIG_UNIT_FEATURE_DSP_USB,
                },
                .iSelector              = 0,
            },

            /* Processing Unit (AGC for DSP) */
            .agcUnit = {
                .bLength                = USBD_SIZE_AUDIO_ST_PROCESSING_UNIT(1,1),
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_PROCESSING_UNIT,
                .bUnitID                = USBCONFIG_UNIT_PROCESSING_DSP,
                .wProcessType           = USBAC_PU_DYN_RANGE_COMP_PROCESS,
                .bNrInPins              = 1,
                .baSourceID             = {USBCONFIG_UNIT_SELECTOR_DSP,},
                .bNrChannels            = 1,
                .wChannelConfig         = 0x0004,   /* D2: Center Front (C) */
                .iChannelNames          = 0,
                .bControlSize           = 1,
            #if 0
                .bmControls             = {0x3F,},  /* D0: Enable Processing
                                                     * D1: Compression Ratio
                                                     * D2: Max Amplitude
                                                     * D3: Threshold
                                                     * D4: Attack Time
                                                     * D5: Release Time
                                                     */
            #else
                .bmControls             = {0x30,},  /* D4: Attack Time
                                                     * D5: Release Time
                                                     */
            #endif
                .iProcessing            = USBSTR_COMPRESSOR,
            },

            /* Output Terminal DSP */
            .outputTerminal = {
                .bLength                = USBD_SIZE_AUDIO_ST_OUTPUT_TERMINAL,
                .bDescriptorType        = USBD_CS_INTERFACE,
                .bDescriptorSubtype     = USBD_AUDIO_ST_OUTPUT_TERMINAL,
                .bTerminalID            = USBCONFIG_UNIT_TERMINAL_OUT_DSP,
                .wTerminalType          = USBAUDIO_TERMINAL_USB_STREAMING,
                .bAssocTerminal         = 0,        /* no association */
                .bSourceID              = USBCONFIG_UNIT_PROCESSING_DSP,
                .iTerminal              = USBSTR_DEMOD_OUTPUT_TERMINAL,
            },
        },
    },

    .dspStreamingInterface_alt0 = {
        /* Standard AudioStreaming Interface 2 (alternate setting #0) */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_DSP_STREAMING,
            .bAlternateSetting      = 0,
            .bNumEndpoints          = 0,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = 0,
        },
    },

    .dspStreamingInterface_alt1 = {
        /* Standard AudioStreaming Interface 2 (alternate setting #1) */
        .interface = {
            .bLength                = USBD_SIZE_INTERFACE,
            .bDescriptorType        = USBD_INTERFACE,
            .bInterfaceNumber       = USBCONFIG_INTERFACE_AUDIO_DSP_STREAMING,
            .bAlternateSetting      = 1,
            .bNumEndpoints          = 1,
            .bInterfaceClass        = USBC_AUDIO,
            .bInterfaceSubClass     = USBCS_AUDIO_AUDIOSTREAMING,
            .bInterfaceProtocol     = 0,
            .iInterface             = 0,
        },

        /* Audio Stream Audio Class */
        .streamingInterface = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 1,        /* AS_GENERAL */
            .bTerminalLink          = USBCONFIG_UNIT_TERMINAL_OUT_DSP,
            .bDelay                 = 1,
            .wFormatTag             = USBAUDIO_FORMATTAG_PCM,
        },

        /* Format Type Audio */
        .formatType = {
            .bLength                = 11,
            .bDescriptorType        = USBD_CS_INTERFACE,
            .bDescriptorSubtype     = 2,        /* FORMAT_TYPE */
            .bFormatType            = USBAUDIO_FORMAT_TYPE_I,
            .bNrChannels            = 1,
            .bSubFrameSize          = 2,
            .bBitResolution         = 16,
            .bSamFreqType           = 1,        /* 1 frequency follows */
            .tSamFreq               = {
                { LE24(48000ul), },
            },
        },

        /* Audio streaming IN endpoint */
        .endpoint = {
            .bLength                = USBD_SIZE_ENDPOINT_ISO,
            .bDescriptorType        = USBD_ENDPOINT,
            .bEndpointAddress       = USBCONFIG_STREAMING_EP2,
            .bmAttributes           = 0x05,     /* isochronous, async, data */
            .wMaxPacketSize         = USBCONFIG_ISOC_SIZE2,
            .bInterval              = 1,
            .bRefresh               = 0,
            .bSyncAddress           = 0,
        },

        /* Class specific audio endpoint */
        .dataEndpoint = {
            .bLength                = 7,
            .bDescriptorType        = USBD_CS_ENDPOINT,
            .bDescriptorSubtype     = 1,        /* EP_GENERAL */
            .bmAttributes           = 1,        /* Sample rate control */
            .bLockDelayUnits        = 0,
            .wLockDelay             = 0,
        },
    },
  #endif
#endif  /* #if USBCONFIG_SOUNDCARD */
};


/** Array with all configurations (we have only one). */
static const USB_ConfigurationArrayElement fifisdr_configurations_96k [USBCONFIG_NUM_CONFIGURATIONS] = {
    {sizeof(fifisdr_configuration1_96k), &fifisdr_configuration1_96k, },
};
static const USB_ConfigurationArrayElement fifisdr_configurations_192k [USBCONFIG_NUM_CONFIGURATIONS] = {
    {sizeof(fifisdr_configuration1_192k), &fifisdr_configuration1_192k, },
};


/** Complete collection of all FiFi-SDR descriptors */
const USB_DescriptorList fifisdr_descriptors_96k = {
    .device                 = &fifisdr_deviceDescriptor,
    .numConfigurations      = USBCONFIG_NUM_CONFIGURATIONS,
    .callbackGetString      = fifisdr_getString,
    .configurations         = fifisdr_configurations_96k,
};
const USB_DescriptorList fifisdr_descriptors_192k = {
    .device                 = &fifisdr_deviceDescriptor,
    .numConfigurations      = USBCONFIG_NUM_CONFIGURATIONS,
    .callbackGetString      = fifisdr_getString,
    .configurations         = fifisdr_configurations_192k,
};



/** Do initialization for the USB descriptors. */  //TODO reduce stack usage
void fifisdr_initDescriptors (void)
{
    uint32_t command[5];
    uint32_t result[5];
    typedef void (*IAP)(uint32_t [], uint32_t[]);
    uint8_t *p;
    uint8_t i, j;
    uint8_t digit;


    /* Read the serial number of the LPC, and transform it into a USB string descriptor.
     * Issue IAP command 58: Read device serial number
     */
    command[0] = 58;
    ((IAP)0x1FFF1FF1)(command, result);

    /* Make a string out of it */
    p = (uint8_t *)&str_iSerialNumber;
    *p++ = sizeof(str_iSerialNumber);
    *p++ = USBD_STRING;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 8; j++) {
            digit = (result[4-i] >> 28) & 0x0F;
            if (digit < 10) {
                *p++ = '0' + digit;
            }
            else {
                *p++ = '7' + digit;
            }
            *p++ = 0;
            result[4-i] <<= 4;
        }
    }
}




