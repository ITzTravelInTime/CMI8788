/* SPDX-License-Identifier: GPL-2.0 */
#ifndef OXYGEN_H_INCLUDED
#define OXYGEN_H_INCLUDED

#include <architecture/i386/pio.h>
//^ anyone who knows anything will quickly tell you THIS, my friends, THIS header is the straw
// that stirs the MFin DRANK. i call it the "god header"

#include "oxygen_regs.h"
#include <IOKit/IOLib.h>
#include <sys/errno.h>
#include <machine/limits.h>
#include <sys/time.h>
#include <IOKit/pci/IOPCIDevice.h>

#define CF_EXCLUDE_CSTD_HEADERS         1
#define CF_OPEN_SOURCE                  1
typedef unsigned long                   ByteCount;
typedef unsigned long                   ItemCount;
#include <AssertMacros.h>
#include <CoreMIDI/CoreMIDI.h>

#define __APPUL__                       __APPLE__

#define dev_err(dev, format, args...)   do {printf("CMI8788: " format, ##args);} while (0)

/* 1 << PCM_x == OXYGEN_CHANNEL_x */
#define PCM_A                           0
#define PCM_B                           1
#define PCM_C                           2
#define PCM_SPDIF                       3
#define PCM_MULTICH                     4
#define PCM_AC97                        5
#define PCM_COUNT                       6
#define PCI_SUBSYSTEM_ID                0x2e
#define PCI_SUBSYSTEM_VENDOR_ID         0x2c

#define OXYGEN_MCLKS(f_single, f_double, f_quad) ((MCLK_##f_single << 0) | \
(MCLK_##f_double << 2) | \
(MCLK_##f_quad   << 4))

#define OXYGEN_IO_SIZE                  0x100
#define OXYGEN_EEPROM_ID                0x434d    /* "CM" */

/* model-specific configuration of outputs/inputs */
#define PLAYBACK_0_TO_I2S               0x0001
/* PLAYBACK_0_TO_AC97_0        not implemented */
#define PLAYBACK_1_TO_SPDIF             0x0004
#define PLAYBACK_2_TO_AC97_1            0x0008
#define CAPTURE_0_FROM_I2S_1            0x0010
#define CAPTURE_0_FROM_I2S_2            0x0020
/* CAPTURE_0_FROM_AC97_0        not implemented */
#define CAPTURE_1_FROM_SPDIF            0x0080
#define CAPTURE_2_FROM_I2S_2            0x0100
#define CAPTURE_2_FROM_AC97_1           0x0200
#define CAPTURE_3_FROM_I2S_3            0x0400

#define CAPTURE_SRC_MIC                 0
#define CAPTURE_SRC_FP_MIC              1
#define CAPTURE_SRC_LINE                2
#define CAPTURE_SRC_AUX                 3

#define PLAYBACK_DST_HP                 0
#define PLAYBACK_DST_HP_FP              1
#define PLAYBACK_DST_MULTICH            2


#define MIDI_OUTPUT                     0x0800
#define MIDI_INPUT                      0x1000
#define AC97_CD_INPUT                   0x2000
#define AC97_FMIC_SWITCH                0x4000
#define MSEC_PER_SEC                    1000L
#define HZ                              1024
//#define BUFFER_SIZE            (NUM_SAMPLE_FRAMES * NUM_CHANNELS * BIT_DEPTH / 8)


#define PCM1796_REG_BASE                16

#define MODEL_HDAV                      0x8314
#define MODEL_ST                        0x835d
#define MODEL_STX                       0x835c
#define MODEL_STX2                      0x85f4
#define MODEL_D2                        0x8269
#define MODEL_D2X                       0x82b7
#define MODEL_XENSE                     0x8428
#define XONAR_GENERIC                   7

#define MODEL_D1                        0x834F
#define MODEL_CS43XX                    0x8275
#define MODEL_DX                        0x8327

#define MODEL_DS                        0x838e
#define MODEL_DSX                       0x8522
#define HDAV_SLIM                       0x835e

#define MODEL_MERIDIAN                  0x5431
#define MODEL_MERIDIAN_2G               0x017a
#define MODEL_CLARO                     0x9761
#define MODEL_CLARO_HALO                0x9781
#define MODEL_FANTASIA                  0x1710
#define MODEL_SERENADE                  0x1711
#define MODEL_2CH_OUTPUT                0x8782
#define MODEL_HG2PCI                    0xffff
#define MODEL_XONAR_DG                  0x8467
#define MODEL_XONAR_DGX                 0x8521

#define OXYGEN_REC_A_OPS                0x0001
#define OXYGEN_REC_B_OPS                0x0002
#define OXYGEN_REC_C_OPS                0x0003
#define OXYGEN_SPDIF_OPS                0x0004
#define OXYGEN_MULTICH_OPS              0x0005
#define OXYGEN_AC97_OPS                 0x0006

#define GPIO_AK5385_DFS_MASK            0x0003
#define GPIO_AK5385_DFS_NORMAL          0x0000
#define GPIO_AK5385_DFS_DOUBLE          0x0001
#define GPIO_AK5385_DFS_QUAD            0x0002

#define GPIO_MERIDIAN_DIG_MASK          0x0050
#define GPIO_MERIDIAN_DIG_EXT           0x0010
#define GPIO_MERIDIAN_DIG_BOARD         0x0040

#define GPIO_CLARO_DIG_COAX             0x0040
#define GPIO_CLARO_HP                   0x0100

#define GPIO_MAGIC                      0x0008
#define GPIO_HP_DETECT                  0x0010
#define GPIO_HP_REAR                    0x0080
#define GPIO_OUTPUT_ENABLE              0x0100


#define GPIO_CS53x1_M_MASK              0x000c
#define GPIO_CS53x1_M_SINGLE            0x0000
#define GPIO_CS53x1_M_DOUBLE            0x0004
#define GPIO_CS53x1_M_QUAD              0x0008
#define XONAR_GPIO_BIT_INVERT           (1 << 16)
#define GPIO_D2X_EXT_POWER              0x0020
#define GPIO_D2_ALT                     0x0080
#define GPIO_D2_OUTPUT_ENABLE           0x0100

#define GPI_EXT_POWER                   0x01
#ifdef XonarGenericAudioEngine_hpp
#define GPIO_INPUT_ROUTE                0x0060 //xonar_dg def
#else
#define GPIO_INPUT_ROUTE                0x0100 // PCM_179x devices
#endif
#define GPIO_HDAV_OUTPUT_ENABLE         0x0001
#define GPIO_HDAV_MAGIC                 0x00c0

#define GPIO_DB_MASK                    0x0030
#define GPIO_DB_H6                      0x0000

#define GPIO_ST_OUTPUT_ENABLE           0x0001
#define GPIO_ST_HP_REAR                 0x0002
#define GPIO_ST_MAGIC                   0x0040
#define GPIO_ST_HP                      0x0080

#define GPIO_XENSE_OUTPUT_ENABLE        (0x0001 | 0x0010 | 0x0020)
#define GPIO_XENSE_SPEAKERS             0x0080

#define I2C_DEVICE_PCM1796(i)           (0x98 + ((i) << 1))    /* 10011, ii, /W=0 */
#define I2C_DEVICE_CS2000               0x9c            /* 100111, 0, /W=0 */

#define I2C_DEVICE_CS4398               0x9e    /* 10011, AD1=1, AD0=1, /W=0 */
#define I2C_DEVICE_CS4362A              0x30    /* 001100, AD0=0, /W=0 */


/* the below has been taken fro oxygen_pcm.c*/
/* most DMA channels have a 16-bit counter for 32-bit words */
#define BUFFER_BYTES_MAX                ((1 << 16) * 4)
/* the multichannel DMA channel has a 24-bit counter */
#define BUFFER_BYTES_MAX_MULTICH        ((1 << 24) * 4)

#define FIFO_BYTES                      256
#define FIFO_BYTES_MULTICH              1024

#define PERIOD_BYTES_MIN                64

#define DEFAULT_BUFFER_BYTES            (BUFFER_BYTES_MAX / 2)
#define DEFAULT_BUFFER_BYTES_MULTICH    (1024 * 1024)
/*end oxygen_pcm.c macro def'ns */


#define likely(x)                       __builtin_expect((x),1)
#define unlikely(x)                     __builtin_expect((x),0)


// lets use bitmasking so we don't have to do double-for loops

#define SNDRV_PCM_RATE_5512             (1<<0)        /* 5512Hz */
#define SNDRV_PCM_RATE_8000             (1<<1)        /* 8000Hz */
#define SNDRV_PCM_RATE_11025            (1<<2)        /* 11025Hz */
#define SNDRV_PCM_RATE_16000            (1<<3)        /* 16000Hz */
#define SNDRV_PCM_RATE_22050            (1<<4)        /* 22050Hz */
#define SNDRV_PCM_RATE_32000            (1<<5)        /* 32000Hz */
#define SNDRV_PCM_RATE_44100            (1<<6)        /* 44100Hz */
#define SNDRV_PCM_RATE_48000            (1<<7)        /* 48000Hz */
#define SNDRV_PCM_RATE_64000            (1<<8)        /* 64000Hz */
#define SNDRV_PCM_RATE_88200            (1<<9)        /* 88200Hz */
#define SNDRV_PCM_RATE_96000            (1<<10)        /* 96000Hz */
#define SNDRV_PCM_RATE_176400           (1<<11)        /* 176400Hz */
#define SNDRV_PCM_RATE_192000           (1<<12)        /* 192000Hz */

//objective C does not use the bitwise operator
// so i am simply declaring SNDRV_PCM_FORMAT as-is
//Linux Def below for reference:
//typedef int __bitwise snd_pcm_format_t;
//#define    SNDRV_PCM_FORMAT_S16_LE    ((__force snd_pcm_format_t) 2)
//OSX Def (may be wrong, but may not be [not sure about significance of bitwise here]):
#define SNDRV_PCM_FORMAT_S16_LE         2

class PCIAudioDevice;
class XonarAudioEngine;

enum {
        CONTROL_SPDIF_PCM,
        CONTROL_SPDIF_INPUT_BITS,
        CONTROL_MIC_CAPTURE_SWITCH,
        CONTROL_LINE_CAPTURE_SWITCH,
        CONTROL_CD_CAPTURE_SWITCH,
        CONTROL_AUX_CAPTURE_SWITCH,
        CONTROL_COUNT
};


struct generic_data {
        unsigned int dacs;
        UInt8 ak4396_regs[4][5];
        UInt16 wm8785_regs[3];
} __attribute__((aligned(8)));

struct snd_info_buffer;
struct timespec;


static const unsigned int channel_base_registers[PCM_COUNT] = {
        [PCM_A] = OXYGEN_DMA_A_ADDRESS,
        [PCM_B] = OXYGEN_DMA_B_ADDRESS,
        [PCM_C] = OXYGEN_DMA_C_ADDRESS,
        [PCM_SPDIF] = OXYGEN_DMA_SPDIF_ADDRESS,
        [PCM_MULTICH] = OXYGEN_DMA_MULTICH_ADDRESS,
        [PCM_AC97] = OXYGEN_DMA_AC97_ADDRESS,
};

struct oxygen_model {
        
        const unsigned int *dac_tlv;
        size_t model_data_size;
        const char *shortname;
        unsigned int device_config;
        
        UInt8 dac_channels_pcm;
        UInt8 dac_channels_mixer;
        UInt8 dac_volume_min;
        UInt8 dac_volume_max;
        UInt8 misc_flags;
        UInt8 function_flags;
        UInt8 dac_mclks;
        UInt8 adc_mclks;
        UInt16 dac_i2s_format;
        UInt16 adc_i2s_format;
        void (*init)(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        int (*control_filter)(struct snd_kcontrol_new *_template);
        int (*mixer_init)(struct oxygen *chip, PCIAudioDevice *dev, XonarAudioEngine *audioEngineInstance);
        void (*resume)(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        void (*cleanup)(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        void (*suspend)(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        void (*pcm_hardware_filter)(unsigned int channel,
                                    IOAudioStream *hardware);
        void (*set_dac_params)(struct oxygen *chip, XonarAudioEngine *audioEngineInstance, IOAudioStream *currentStream);
        void (*set_adc_params)(struct oxygen *chip,
                               XonarAudioEngine *audioEngineInstance);
        void (*update_dac_volume)(struct oxygen *chip);
        void (*update_dac_mute)(struct oxygen *chip);
        void (*update_center_lfe_mix)(struct oxygen *chip, bool mixed);
        unsigned int (*adjust_dac_routing)(struct oxygen *chip,
                                           unsigned int play_routing);
        void (*gpio_changed)(struct oxygen *chip, XonarAudioEngine *audioEngineInstance);
        void (*uart_input)(struct oxygen *chip);
        void (*ac97_switch)(struct oxygen *chip,
                            unsigned int reg, unsigned int mute);
        void (*dump_registers)(struct oxygen *chip,
                               snd_info_buffer *buffer);
} __attribute__((aligned(8)));


struct oxygen {
//        unsigned long addr;
        IOMemoryMap *addr;
        IOPCIDevice *dev;
        IOSimpleLock *reg_lock;
        IOLock *mutex;
        IOLock *ac97_mutex;
        //    struct snd_card *card;
        //    struct pci_dev *pci;
        //    struct snd_rawmidi *midi;
#ifdef HAVE_MPU401
        //if we ever get there
        MIDIDeviceRef               midi;
        MIDIDriverInterface *    mInterface;        // keep this first
        MIDIDriverRef        Self() { return &mInterface; };
        MIDIPortRef     midiOut;
        MIDIPortRef     midiIn;
#endif
        int irq;
        UInt16 card_model;
        void *model_data;
        unsigned int interrupt_mask;
        UInt8 dac_volume[8];
        UInt8 dac_mute;
        UInt8 pcm_active;
        UInt8 pcm_running;
        UInt8 dac_routing;
        UInt8 spdif_playback_enable;
        UInt8 has_ac97_0;
        UInt8 has_ac97_1;
        UInt32 spdif_bits;
        UInt32 spdif_pcm_bits;
        IOAudioStream *streams[PCM_COUNT];
        //   struct snd_pcm_substream *streams[PCM_COUNT];
        IOAudioControl *controls[CONTROL_COUNT];
        //IOWorkLoop spdif_input_bits_work;
        //IOWorkLoop gpio_work;
        //wait_queue_t ac97_waitqueue;
        //pthread_cond_t  ac97_condition = PTHREAD_COND_INITIALIZER;
        bool ac97_statusbits;
        struct timespec ac97_timeout = {0, (long)1e6};
        wait_result_t ac97_result;
        unsigned int ac97_maskval;
        union {// have to swap these ... remember.
                UInt8 _8[OXYGEN_IO_SIZE];
                UInt16 _16[OXYGEN_IO_SIZE / 2];
                UInt32 _32[OXYGEN_IO_SIZE / 4];
        } saved_registers;
        UInt16 saved_ac97_registers[2][0x40];
        unsigned int uart_input_count;
        UInt8 uart_input[32];
        struct oxygen_model model;
} __attribute__((aligned(8)));

struct xonar_generic {
        unsigned int anti_pop_delay;
        UInt16 output_enable_bit;
        UInt8 ext_power_reg;
        UInt8 ext_power_int_reg;
        UInt8 ext_power_bit;
        UInt8 has_power;
};

struct xonar_hdmi {
        UInt8 params[5];
};

struct xonar_pcm179x {
        struct xonar_generic generic;
        unsigned int dacs;
        UInt8 pcm1796_regs[4][5];
        IOAudioSampleRate current_rate;
        bool h6;
        bool hp_active;
        SInt8 hp_gain_offset;
        bool has_cs2000;
        UInt8 cs2000_regs[0x1f];
        bool broken_i2c;
};

struct xonar_hdav {
        struct xonar_pcm179x pcm179x;
        struct xonar_hdmi hdmi;
};


struct xonar_cs43xx {
        struct xonar_generic generic;
        UInt8 cs4398_regs[8];
        UInt8 cs4362a_regs[15];
};



struct xonar_wm87x6 {
        struct xonar_generic generic;
        UInt16 wm8776_regs[0x17];
        UInt16 wm8766_regs[0x10];
        //struct snd_kcontrol *line_adcmux_control;
        //struct snd_kcontrol *mic_adcmux_control;
        //struct snd_kcontrol *lc_controls[13];
        //struct snd_jack *hp_jack;
        struct xonar_hdmi hdmi;
};

struct dg {
        /* shadow copy of the CS4245 register space */
        unsigned char cs4245_shadow[17];
        /* output select: headphone/speakers */
        unsigned char output_sel;
        /* volumes for all capture sources */
        char input_vol[4][2];
        /* input select: mic/fp mic/line/aux */
        unsigned char input_sel;
};

static inline UInt8 oxygen_read8(struct oxygen *chip, unsigned int reg)
{
#if defined(__APPUL__)
        return chip->dev->ioRead8(reg, chip->addr);
  //      return IOMappedRead8(chip->addr + reg);
#else
        return inb(chip->addr + reg);
#endif
}
//EXPORT_SYMBOL(oxygen_read8);

static inline UInt16 oxygen_read16(struct oxygen *chip, unsigned int reg)
{
#if defined(__APPUL__)
        return chip->dev->ioRead16(reg, chip->addr);
//        return IOMappedRead16(chip->addr + reg);
#else
        return inw(chip->addr + reg);
#endif
}
//EXPORT_SYMBOL(oxygen_read16);

static inline UInt32 oxygen_read32(struct oxygen *chip, unsigned int reg)
{
#if defined(__APPUL__)
        return chip->dev->ioRead32(reg, chip->addr);
//        return IOMappedRead32(chip->addr + reg);
#else
        return inl(chip->addr + reg);
#endif
}
////EXPORT_SYMBOL(oxygen_read32);

static inline void oxygen_write8(struct oxygen *chip, unsigned int reg, UInt8 value)
{
#if defined(__APPUL__)
          chip->dev->ioWrite8(reg, value, chip->addr);
     //   IOMappedWrite8(chip->addr + reg, value);
#else
        outb(value, chip->addr + reg);
#endif
        chip->saved_registers._8[reg] = value;
}
//EXPORT_SYMBOL(oxygen_write8);

static inline void oxygen_write16(struct oxygen *chip, unsigned int reg, UInt16 value)
{
#if defined(__APPUL__)
        chip->dev->ioWrite16(reg, value, chip->addr);
      //  IOMappedWrite16(chip->addr + reg, value);
#else
        outw(value, chip->addr + reg);
#endif
        chip->saved_registers._16[reg / 2] = OSSwapHostToLittleInt16(value);
}
//EXPORT_SYMBOL(oxygen_write16);

static inline void oxygen_write32(struct oxygen *chip, unsigned int reg, UInt32 value)
{
#if defined(__APPUL__)
        chip->dev->ioWrite32(reg, value, chip->addr);
     //   IOMappedWrite32(chip->addr +reg, value);
#else
                outl(value, chip->addr + reg);
#endif
        chip->saved_registers._32[reg / 4] = OSSwapHostToLittleInt32(value);
}
//EXPORT_SYMBOL(oxygen_write32);

static inline void oxygen_write8_masked(struct oxygen *chip, unsigned int reg,
                                        UInt8 value, UInt8 mask)
{
        UInt8 tmp;
#if defined(__APPUL__)
        tmp = chip->dev->ioRead8(reg, chip->addr);
        //tmp = IOMappedRead8(chip->addr + reg);
#else
        tmp = inb(chip->addr + reg);
#endif
        
        tmp &= ~mask;
        tmp |= value & mask;
#if defined(__APPUL__)
        chip->dev->ioWrite8(reg, tmp, chip->addr);
        //IOMappedWrite8(chip->addr + reg, tmp);
#else
        outb(tmp, chip->addr + reg);
#endif
        chip->saved_registers._8[reg] = tmp;
}
//EXPORT_SYMBOL(oxygen_write8_masked);

static inline void oxygen_write16_masked(struct oxygen *chip, unsigned int reg,
                                         UInt16 value, UInt16 mask)
{
        UInt16 tmp;
#if defined(__APPUL__)
        tmp = chip->dev->ioRead16(reg, chip->addr);
        //tmp = IOMappedRead16(chip->addr + reg);
#else
        tmp = inw(chip->addr + reg);
#endif
        tmp &= ~mask;
        tmp |= value & mask;
#if defined(__APPUL__)
        chip->dev->ioWrite16(reg, tmp, chip->addr);
        //IOMappedWrite16(chip->addr + reg, tmp);
#else
        outw(tmp, chip->addr + reg);
#endif
        chip->saved_registers._16[reg / 2] = OSSwapHostToLittleInt16(tmp);
}
//EXPORT_SYMBOL(oxygen_write16_masked);

static inline void oxygen_write32_masked(struct oxygen *chip, unsigned int reg,
                                         UInt32 value, UInt32 mask)
{
        UInt32 tmp;
#if defined(__APPUL__)
        tmp = chip->dev->ioRead32(reg, chip->addr);
        //tmp = IOMappedRead32(chip->addr + reg);
#else
        tmp = inl(chip->addr + reg);
#endif
        tmp &= ~mask;
        tmp |= value & mask;
#if defined(__APPUL__)
        chip->dev->ioWrite32(reg, tmp, chip->addr);
        //IOMappedWrite32(chip->addr + reg, tmp);
#else
        outl(tmp, chip->addr + reg);
#endif
        chip->saved_registers._32[reg / 4] = OSSwapHostToLittleInt32(tmp);
}
//EXPORT_SYMBOL(oxygen_write32_masked);


static inline void oxygen_set_bits8(struct oxygen *chip,
                                    unsigned int reg, UInt8 value)
{
        oxygen_write8_masked(chip, reg, value, value);
}

static inline void oxygen_set_bits16(struct oxygen *chip,
                                     unsigned int reg, UInt16 value)
{
        oxygen_write16_masked(chip, reg, value, value);
}

static inline void oxygen_set_bits32(struct oxygen *chip,
                                     unsigned int reg, UInt32 value)
{
        oxygen_write32_masked(chip, reg, value, value);
}

static inline void oxygen_clear_bits8(struct oxygen *chip,
                                      unsigned int reg, UInt8 value)
{
        oxygen_write8_masked(chip, reg, 0, value);
}

static inline void oxygen_clear_bits16(struct oxygen *chip,
                                       unsigned int reg, UInt16 value)
{
        oxygen_write16_masked(chip, reg, 0, value);
}

static inline void oxygen_clear_bits32(struct oxygen *chip,
                                       unsigned int reg, UInt32 value)
{
        oxygen_write32_masked(chip, reg, 0, value);
}

static inline void oxygen_write_i2c(struct oxygen *chip, UInt8 device, UInt8 map, UInt8 data)
{
        /* should not need more than about 300 us */
        IODelay(1000);
        
        oxygen_write8(chip, OXYGEN_2WIRE_MAP, map);
        oxygen_write8(chip, OXYGEN_2WIRE_DATA, data);
        oxygen_write8(chip, OXYGEN_2WIRE_CONTROL,
                      device | OXYGEN_2WIRE_DIR_WRITE);
}


static inline int oxygen_wait_spi(struct oxygen *chip)
{
        unsigned int count;
        
        /*
         * Higher timeout to be sure: 200 us;
         * actual transaction should not need more than 40 us.
         */
        for (count = 50; count > 0; count--) {
                IODelay(4);
                if ((oxygen_read8(chip, OXYGEN_SPI_CONTROL) &
                     OXYGEN_SPI_BUSY) == 0)
                        return 0;
        }
        kprintf("oxygen: SPI wait timeout\n");
        return -EIO;
}

static inline int oxygen_write_spi(struct oxygen *chip, UInt8 control, unsigned int data)
{
        /*
         * We need to wait AFTER initiating the SPI transaction,
         * otherwise read operations will not work.
         */
        oxygen_write8(chip, OXYGEN_SPI_DATA1, data);
        oxygen_write8(chip, OXYGEN_SPI_DATA2, data >> 8);
        if (control & OXYGEN_SPI_DATA_LENGTH_3)
                oxygen_write8(chip, OXYGEN_SPI_DATA3, data >> 16);
        oxygen_write8(chip, OXYGEN_SPI_CONTROL, control);
        return oxygen_wait_spi(chip);
}

static inline int is_bit_set(const UInt32 *bitmap, unsigned int bit)
{
        return bitmap[bit / 32] & (1 << (bit & 31));
}

//because other files use these functions below, we put them here.
//similar argument goes for the header, since it's only used by the
//two functions below.


#include "cs4245.h"


enum cs4245_shadow_operation {
        CS4245_SAVE_TO_SHADOW,
        CS4245_LOAD_FROM_SHADOW
};


inline int cs4245_write_spi(struct oxygen *chip, UInt8 reg)
{
        struct dg *data = (struct dg*) chip->model_data;
        unsigned int packet;
        
        packet = reg << 8;
        packet |= (CS4245_SPI_ADDRESS | CS4245_SPI_WRITE) << 16;
        packet |= data->cs4245_shadow[reg];
        
        return oxygen_write_spi(chip, OXYGEN_SPI_TRIGGER |
                                OXYGEN_SPI_DATA_LENGTH_3 |
                                OXYGEN_SPI_CLOCK_1280 |
                                (0 << OXYGEN_SPI_CODEC_SHIFT) |
                                OXYGEN_SPI_CEN_LATCH_CLOCK_HI,
                                packet);
}
/* Capture Source */

inline int input_source_apply(struct oxygen *chip)
{
        struct dg *data = (struct dg*) chip->model_data;
        
        data->cs4245_shadow[CS4245_ANALOG_IN] &= ~CS4245_SEL_MASK;
        if (data->input_sel == CAPTURE_SRC_FP_MIC)
                data->cs4245_shadow[CS4245_ANALOG_IN] |= CS4245_SEL_INPUT_2;
        else if (data->input_sel == CAPTURE_SRC_LINE)
                data->cs4245_shadow[CS4245_ANALOG_IN] |= CS4245_SEL_INPUT_4;
        else if (data->input_sel != CAPTURE_SRC_MIC)
                data->cs4245_shadow[CS4245_ANALOG_IN] |= CS4245_SEL_INPUT_1;
        return cs4245_write_spi(chip, CS4245_ANALOG_IN);
}



#endif
