/*
 File:SamplePCIAudioDevice.h
 
 Contains:
 
 Version:1.0.0
 
 Copyright:Copyright ) 1997-2000 by Apple Computer, Inc., All Rights Reserved.
 
 Disclaimer:IMPORTANT:  This Apple software is supplied to you by Apple Computer, Inc.
 ("Apple") in consideration of your agreement to the following terms, and your use,
 installation, modification or redistribution of this Apple software constitutes acceptance
 of these terms.  If you do not agree with these terms, please do not use, install, modify or
 redistribute this Apple software.
 
 In consideration of your agreement to abide by the following terms, and subject
 to these terms, Apple grants you a personal, non-exclusive license, under Apple's
 copyrights in this original Apple software (the "Apple Software"), to use, reproduce,
 modify and redistribute the Apple Software, with or without modifications, in source and/or
 binary forms; provided that if you redistribute the Apple Software in its entirety
 and without modifications, you must retain this notice and the following text
 and disclaimers in all such redistributions of the Apple Software.  Neither the
 name, trademarks, service marks or logos of Apple Computer, Inc. may be used to
 endorse or promote products derived from the Apple Software without specific prior
 written permission from Apple.  Except as expressly stated in this notice, no
 other rights or licenses, express or implied, are granted by Apple herein,
 including but not limited to any patent rights that may be infringed by your derivative
 works or by other works in which the Apple Software may be incorporated.
 
 The Apple Software is provided by Apple on an "AS IS" basis.  APPLE MAKES NO WARRANTIES,
 EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT,
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE APPLE SOFTWARE
 OR ITS USE AND OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL APPLE
 BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
 REPRODUCTION, MODIFICATION AND/OR DISTRIBUTION OF THE APPLE SOFTWARE, HOWEVER CAUSED
 AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE), STRICT
 LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 */


#ifndef _SAMPLEPCIAUDIODEVICE_H
#define _SAMPLEPCIAUDIODEVICE_H
#define _POSIX_C_SOURCE // to remove conflict with mach_port_t (happens when you use pthreads)
#include <IOKit/audio/IOAudioDevice.h>
#include <IOKit/IOWorkLoop.h>
//#include <IOKit/IOLocks.h>
#include <sys/types.h>
//#include <mach/semaphore.h>
//#include <mach/task.h>
#include <architecture/i386/pio.h>
#include <sys/errno.h>
#include <machine/limits.h>
//#include <kern/waitq.h>
#include "oxygen_regs.h"
//#include <libkern/OSAtomic.h>
#include </usr/include/pthread.h>
#define dev_err(dev, format, args...) do {IOLog("CMI8788: " format, ##args);} while (0)

/* 1 << PCM_x == OXYGEN_CHANNEL_x */
#define PCM_A		0
#define PCM_B		1
#define PCM_C		2
#define PCM_SPDIF	3
#define PCM_MULTICH	4
#define PCM_AC97	5
#define PCM_COUNT	6
#define PCI_SUBSYSTEM_ID	0x2e
#define PCI_SUBSYSTEM_VENDOR_ID	0x2c

#define OXYGEN_MCLKS(f_single, f_double, f_quad) ((MCLK_##f_single << 0) | \
(MCLK_##f_double << 2) | \
(MCLK_##f_quad   << 4))

#define OXYGEN_IO_SIZE	0x100

#define OXYGEN_EEPROM_ID	0x434d	/* "CM" */

/* model-specific configuration of outputs/inputs */
#define PLAYBACK_0_TO_I2S	0x0001
/* PLAYBACK_0_TO_AC97_0		not implemented */
#define PLAYBACK_1_TO_SPDIF	0x0004
#define PLAYBACK_2_TO_AC97_1	0x0008
#define CAPTURE_0_FROM_I2S_1	0x0010
#define CAPTURE_0_FROM_I2S_2	0x0020
/* CAPTURE_0_FROM_AC97_0		not implemented */
#define CAPTURE_1_FROM_SPDIF	0x0080
#define CAPTURE_2_FROM_I2S_2	0x0100
#define CAPTURE_2_FROM_AC97_1	0x0200
#define CAPTURE_3_FROM_I2S_3	0x0400
#define MIDI_OUTPUT		0x0800
#define MIDI_INPUT		0x1000
#define AC97_CD_INPUT		0x2000
#define AC97_FMIC_SWITCH	0x4000
#define MSEC_PER_SEC        1000L
#define HZ                  1024
#define BUFFER_SIZE			(NUM_SAMPLE_FRAMES * NUM_CHANNELS * BIT_DEPTH / 8)

#define IEC958_AES1_CON_DIGDIGCONV_ID   0x02
#define IEC958_AES1_CON_PCM_CODER       (IEC958_AES1_CON_DIGDIGCONV_ID|0x00)
#define IEC958_AES3_CON_FS_48000	(2<<0)	/* 48kHz */
#define IEC958_AES3_CON_FS_44100        (0<<0)  /* 44.1kHz */
#define IEC958_AES3_CON_FS_96000        (10<<0) /* 96kHz */
#define IEC958_AES3_CON_FS_192000       (14<<0) /* 192kHz */


enum {
    CONTROL_SPDIF_PCM,
    CONTROL_SPDIF_INPUT_BITS,
    CONTROL_MIC_CAPTURE_SWITCH,
    CONTROL_LINE_CAPTURE_SWITCH,
    CONTROL_CD_CAPTURE_SWITCH,
    CONTROL_AUX_CAPTURE_SWITCH,
    CONTROL_COUNT
};


typedef struct SamplePCIAudioDeviceRegisters {
    UInt32	reg1;
    UInt32	reg2;
    UInt32	reg3;
    UInt32	reg4;
} SamplePCIAudioDeviceRegisters;


struct generic_data {
    unsigned int dacs;
    UInt8 ak4396_regs[4][5];
    UInt16 wm8785_regs[3];
};


struct oxygen_model {
    
    const unsigned int *dac_tlv;
    size_t model_data_size;
    unsigned int device_config;
    UInt8 dac_channels_pcm;
    UInt8 dac_channels_mixer;
    UInt8 dac_volume_min;
    UInt8 dac_volume_max;
    void (*uart_input)(struct oxygen *chip);
    UInt8 misc_flags;
    UInt8 function_flags;
    UInt8 dac_mclks;
    UInt8 adc_mclks;
    UInt16 dac_i2s_format;
    UInt16 adc_i2s_format;
    void (*gpio_changed)(struct oxygen *chip);
};

struct oxygen {
    unsigned long addr;
    OSSpinLock reg_lock;
    IOLock *mutex;
    struct snd_card *card;
    struct pci_dev *pci;
    struct snd_rawmidi *midi;
    int irq;
    void *model_data;
    unsigned int interrupt_mask;
    UInt8 dac_volume[8];
    UInt8 dac_mute;
    UInt8 pcm_active;
    UInt8 pcm_running;
    UInt8 dac_routing;
   // void (*gpio_changed)(struct oxygen *chip);
    UInt8 spdif_playback_enable;
    UInt8 has_ac97_0;
    UInt8 has_ac97_1;
    UInt32 spdif_bits;
    UInt32 spdif_pcm_bits;
    IOAudioStreamDataDescriptor *streams[PCM_COUNT];
 //   struct snd_pcm_substream *streams[PCM_COUNT];
 //   struct snd_kcontrol *controls[CONTROL_COUNT];
    IOWorkLoop spdif_input_bits_work;
    IOWorkLoop gpio_work;
    //wait_queue_t ac97_waitqueue;
    pthread_cond_t  ac97_condition;
    pthread_mutex_t  ac97_mutex;
    struct timespec ac97_timeout = {0, (long)1e6};
    union {// have to swap these ... remember.
        UInt8 _8[OXYGEN_IO_SIZE];
        SInt16 _16[OXYGEN_IO_SIZE / 2];
        SInt32 _32[OXYGEN_IO_SIZE / 4];
    } saved_registers;
    UInt16 saved_ac97_registers[2][0x40];
    unsigned int uart_input_count;
    UInt8 uart_input[32];
    struct oxygen_model model;
};



static inline unsigned long msecs_to_jiffies(const unsigned int m)
{
    return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
}

UInt8 oxygen_read8(struct oxygen *chip, unsigned int reg)
{
    return inb(chip->addr + reg);
}
//EXPORT_SYMBOL(oxygen_read8);

UInt16 oxygen_read16(struct oxygen *chip, unsigned int reg)
{
    return inw(chip->addr + reg);
}
//EXPORT_SYMBOL(oxygen_read16);

UInt32 oxygen_read32(struct oxygen *chip, unsigned int reg)
{
    return inl(chip->addr + reg);
}
////EXPORT_SYMBOL(oxygen_read32);

void oxygen_write8(struct oxygen *chip, unsigned int reg, UInt8 value)
{
    outb(value, chip->addr + reg);
    chip->saved_registers._8[reg] = value;
}
//EXPORT_SYMBOL(oxygen_write8);

void oxygen_write16(struct oxygen *chip, unsigned int reg, UInt16 value)
{
    outw(value, chip->addr + reg);
    chip->saved_registers._16[reg / 2] = OSSwapHostToLittleInt16(value);
}
//EXPORT_SYMBOL(oxygen_write16);

void oxygen_write32(struct oxygen *chip, unsigned int reg, UInt32 value)
{
    outl(value, chip->addr + reg);
    chip->saved_registers._32[reg / 4] = OSSwapHostToLittleInt32(value);
}
//EXPORT_SYMBOL(oxygen_write32);

void oxygen_write8_masked(struct oxygen *chip, unsigned int reg,
                          UInt8 value, UInt8 mask)
{
    UInt8 tmp = inb(chip->addr + reg);
    tmp &= ~mask;
    tmp |= value & mask;
    outb(tmp, chip->addr + reg);
    chip->saved_registers._8[reg] = tmp;
}
//EXPORT_SYMBOL(oxygen_write8_masked);

void oxygen_write16_masked(struct oxygen *chip, unsigned int reg,
                           UInt16 value, UInt16 mask)
{
    UInt16 tmp = inw(chip->addr + reg);
    tmp &= ~mask;
    tmp |= value & mask;
    outw(tmp, chip->addr + reg);
    chip->saved_registers._16[reg / 2] = OSSwapHostToLittleInt16(tmp);
}
//EXPORT_SYMBOL(oxygen_write16_masked);

void oxygen_write32_masked(struct oxygen *chip, unsigned int reg,
                           UInt32 value, UInt32 mask)
{
    UInt32 tmp = inl(chip->addr + reg);
    tmp &= ~mask;
    tmp |= value & mask;
    outl(tmp, chip->addr + reg);
    chip->saved_registers._32[reg / 4] = OSSwapHostToLittleInt32(tmp);
}
//EXPORT_SYMBOL(oxygen_write32_masked);

static int oxygen_ac97_wait(struct oxygen *chip, unsigned int mask)
{
    UInt8 status = 0;
    
    /*
     * Reading the status register also clears the bits, so we have to save
     * the read bits in status.
     */

     //wait_queue_assert_wait(chip->ac97_waitqueue,
                         //   (event_t),1);
    pthread_mutex_lock(&chip->>ac97_mutex);
    if(({ status |= oxygen_read8(chip, OXYGEN_AC97_INTERRUPT_STATUS);status & mask;}))
        pthread_cond_timedwait(&chip->ac97_condition,&chip->ac97_mutex,&chip->ac97_timeout);
    pthread_mutex_unlock(&chip->ac97_mutex);
    /*
     * Check even after a timeout because this function should not require
     * the AC'97 interrupt to be enabled.
     */
    status |= oxygen_read8(chip, OXYGEN_AC97_INTERRUPT_STATUS);
    return status & mask ? 0 : -EIO;
}

/*
 * About 10% of AC'97 register reads or writes fail to complete, but even those
 * where the controller indicates completion aren't guaranteed to have actually
 * happened.
 *
 * It's hard to assign blame to either the controller or the codec because both
 * were made by C-Media ...
 */

void oxygen_write_ac97(struct oxygen *chip, unsigned int codec,
                       unsigned int index, UInt16 data)
{
    unsigned int count, succeeded;
    UInt32 reg;
    
    reg = data;
    reg |= index << OXYGEN_AC97_REG_ADDR_SHIFT;
    reg |= OXYGEN_AC97_REG_DIR_WRITE;
    reg |= codec << OXYGEN_AC97_REG_CODEC_SHIFT;
    succeeded = 0;
    for (count = 5; count > 0; --count) {
        IODelay(5);
        oxygen_write32(chip, OXYGEN_AC97_REGS, reg);
        /* require two "completed" writes, just to be sure */
        if (oxygen_ac97_wait(chip, OXYGEN_AC97_INT_WRITE_DONE) >= 0 &&
            ++succeeded >= 2) {
            chip->saved_ac97_registers[codec][index / 2] = data;
            return;
        }
    }
    dev_err(chip->card->dev, "AC'97 write timeout\n");
}
//EXPORT_SYMBOL(oxygen_write_ac97);

UInt16 oxygen_read_ac97(struct oxygen *chip, unsigned int codec,
                        unsigned int index)
{
    unsigned int count;
    unsigned int last_read = UINT_MAX;
    UInt32 reg;
    
    reg = index << OXYGEN_AC97_REG_ADDR_SHIFT;
    reg |= OXYGEN_AC97_REG_DIR_READ;
    reg |= codec << OXYGEN_AC97_REG_CODEC_SHIFT;
    for (count = 5; count > 0; --count) {
        IODelay(5);
        oxygen_write32(chip, OXYGEN_AC97_REGS, reg);
        IODelay(10);
        if (oxygen_ac97_wait(chip, OXYGEN_AC97_INT_READ_DONE) >= 0) {
            UInt16 value = oxygen_read16(chip, OXYGEN_AC97_REGS);
            /* we require two consecutive reads of the same value */
            if (value == last_read)
                return value;
            last_read = value;
            /*
             * Invert the register value bits to make sure that two
             * consecutive unsuccessful reads do not return the same
             * value.
             */
            reg ^= 0xffff;
        }
    }
    dev_err(chip->card->dev, "AC'97 read timeout on codec %u\n", codec);
    return 0;
}
//EXPORT_SYMBOL(oxygen_read_ac97);

void oxygen_write_ac97_masked(struct oxygen *chip, unsigned int codec,
                              unsigned int index, UInt16 data, UInt16 mask)
{
    UInt16 value = oxygen_read_ac97(chip, codec, index);
    value &= ~mask;
    value |= data & mask;
    oxygen_write_ac97(chip, codec, index, value);
}
//EXPORT_SYMBOL(oxygen_write_ac97_masked);



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

static inline void oxygen_ac97_set_bits(struct oxygen *chip, unsigned int codec,
                                        unsigned int index, UInt16 value)
{
    oxygen_write_ac97_masked(chip, codec, index, value, value);
}

static inline void oxygen_ac97_clear_bits(struct oxygen *chip,
                                          unsigned int codec,
                                          unsigned int index, UInt16 value)
{
    oxygen_write_ac97_masked(chip, codec, index, 0, value);
}


class IOPCIDevice;
class IOMemoryMap;

#define PCIAudioDevice com_CMedia_CMI8788_PCIAudioDevice

class PCIAudioDevice : public IOAudioDevice
{
    friend class SampleAudioEngine;
    
    OSDeclareDefaultStructors(PCIAudioDevice)
    
    IOPCIDevice					*pciDevice;
    IOMemoryMap					*deviceMap;
    
    struct oxygen               *deviceRegisters;
    
    virtual bool initHardware(IOService *provider);
    virtual bool createAudioEngine();
    virtual void free();
    
    static IOReturn volumeChangeHandler(IOService *target, IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn volumeChanged(IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue);
    
    static IOReturn outputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn outputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
    
    static IOReturn gainChangeHandler(IOService *target, IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn gainChanged(IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue);
    
    static IOReturn inputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn inputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);

//    int oxygen_pci_probe(struct pci_dev *pci, int index, char *id,
//                         struct module *owner,
//                         const struct pci_device_id *ids,
//                         int (*get_model)(struct oxygen *chip,
//                                          const struct pci_device_id *id
//                                          )
//                         );
    void oxygen_pci_remove(struct pci_dev *pci);
#ifdef CONFIG_PM_SLEEP
    extern const struct dev_pm_ops oxygen_pci_pm;
#endif
    void oxygen_pci_shutdown(struct pci_dev *pci);
    
    /* oxygen_mixer.c */
    
    int oxygen_mixer_init(struct oxygen *chip);
    void oxygen_update_dac_routing(struct oxygen *chip);
    void oxygen_update_spdif_source(struct oxygen *chip);
    
    /* oxygen_pcm.c */
    
    int oxygen_pcm_init(struct oxygen *chip);
    
    /* oxygen_io.c */
    
    UInt8 oxygen_read8(struct oxygen *chip, unsigned int reg);
    UInt16 oxygen_read16(struct oxygen *chip, unsigned int reg);
    UInt32 oxygen_read32(struct oxygen *chip, unsigned int reg);
    void oxygen_write8(struct oxygen *chip, unsigned int reg, UInt8 value);
    void oxygen_write16(struct oxygen *chip, unsigned int reg, UInt16 value);
    void oxygen_write32(struct oxygen *chip, unsigned int reg, UInt32 value);
    void oxygen_write8_masked(struct oxygen *chip, unsigned int reg,
                              UInt8 value, UInt8 mask);
    void oxygen_write16_masked(struct oxygen *chip, unsigned int reg,
                               UInt16 value, UInt16 mask);
    void oxygen_write32_masked(struct oxygen *chip, unsigned int reg,
                               UInt32 value, UInt32 mask);
    
    UInt16 oxygen_read_ac97(struct oxygen *chip, unsigned int codec,
                            unsigned int index);
    void oxygen_write_ac97(struct oxygen *chip, unsigned int codec,
                           unsigned int index, UInt16 data);
    void oxygen_write_ac97_masked(struct oxygen *chip, unsigned int codec,
                                  unsigned int index, UInt16 data, UInt16 mask);
    

    void oxygen_write_i2c(struct oxygen *chip, UInt8 device, UInt8 map, UInt8 data);
    
    
    UInt16 oxygen_read_eeprom(struct oxygen *chip, unsigned int index);
    void oxygen_write_eeprom(struct oxygen *chip, unsigned int index, UInt16 value);
    void oxygen_restore_eeprom(IOPCIDevice *device, struct oxygen *chip);
    void oxygen_init(struct oxygen *chip);
   // const struct * oxygen_search_pci_id(struct oxygen *chip, const struct pci_device_id ids[]);


};

#endif /* _SAMPLEPCIAUDIODEVICE_H */