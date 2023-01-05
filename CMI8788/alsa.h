//
//  alsa.h
//  PCIAudioDriver
//
//  Created by Gagan on 2022-12-04.
//  Copyright © 2022 CMedia. All rights reserved.
//

#ifndef alsa_h
#define alsa_h

#define snd_kcontrol_chip(kcontrol) ((kcontrol)->private_data)

#define SNDRV_CTL_TLVT_CONTAINER 0    /* one level down - group of TLVs */
#define SNDRV_CTL_TLVT_DB_SCALE    1       /* dB scale */
#define SNDRV_CTL_TLVT_DB_LINEAR 2    /* linear volume */
#define SNDRV_CTL_TLVT_DB_RANGE 3    /* dB range container */
#define SNDRV_CTL_TLVT_DB_MINMAX 4    /* dB scale with min/max */
#define SNDRV_CTL_TLVT_DB_MINMAX_MUTE 5    /* dB scale with min/max with mute */

/*
 * channel-mapping TLV items
 *  TLV length must match with num_channels
 */
#define SNDRV_CTL_TLVT_CHMAP_FIXED    0x101    /* fixed channel position */
#define SNDRV_CTL_TLVT_CHMAP_VAR    0x102    /* channels freely swappable */
#define SNDRV_CTL_TLVT_CHMAP_PAIRED    0x103    /* pair-wise swappable */

#define SNDRV_CTL_TLVD_DB_GAIN_MUTE    -9999999

#define SNDRV_CTL_TLVD_ITEM(type, ...) \
    (type), SNDRV_CTL_TLVD_LENGTH(__VA_ARGS__), __VA_ARGS__
#define SNDRV_CTL_TLVD_LENGTH(...) \
    ((unsigned int)sizeof((const unsigned int[]) { __VA_ARGS__ }))

#define SNDRV_CTL_TLVD_CONTAINER_ITEM(...) \
    SNDRV_CTL_TLVD_ITEM(SNDRV_CTL_TLVT_CONTAINER, __VA_ARGS__)
#define SNDRV_CTL_TLVD_DECLARE_CONTAINER(name, ...) \
    unsigned int name[] = { \
        SNDRV_CTL_TLVD_CONTAINER_ITEM(__VA_ARGS__) \
    }

#define SNDRV_CTL_TLVD_DB_SCALE_MASK    0xffff
#define SNDRV_CTL_TLVD_DB_SCALE_MUTE    0x10000
#define SNDRV_CTL_TLVD_DB_SCALE_ITEM(min, step, mute) \
    SNDRV_CTL_TLVD_ITEM(SNDRV_CTL_TLVT_DB_SCALE, \
                (min), \
                ((step) & SNDRV_CTL_TLVD_DB_SCALE_MASK) | \
                 ((mute) ? SNDRV_CTL_TLVD_DB_SCALE_MUTE : 0))
#define SNDRV_CTL_TLVD_DECLARE_DB_SCALE(name, min, step, mute) \
    unsigned int name[] = { \
        SNDRV_CTL_TLVD_DB_SCALE_ITEM(min, step, mute) \
    }

/* dB scale specified with min/max values instead of step */
#define SNDRV_CTL_TLVD_DB_MINMAX_ITEM(min_dB, max_dB) \
    SNDRV_CTL_TLVD_ITEM(SNDRV_CTL_TLVT_DB_MINMAX, (min_dB), (max_dB))
#define SNDRV_CTL_TLVD_DB_MINMAX_MUTE_ITEM(min_dB, max_dB) \
    SNDRV_CTL_TLVD_ITEM(SNDRV_CTL_TLVT_DB_MINMAX_MUTE, (min_dB), (max_dB))
#define SNDRV_CTL_TLVD_DECLARE_DB_MINMAX(name, min_dB, max_dB) \
    unsigned int name[] = { \
        SNDRV_CTL_TLVD_DB_MINMAX_ITEM(min_dB, max_dB) \
    }
#define SNDRV_CTL_TLVD_DECLARE_DB_MINMAX_MUTE(name, min_dB, max_dB) \
    unsigned int name[] = { \
        SNDRV_CTL_TLVD_DB_MINMAX_MUTE_ITEM(min_dB, max_dB) \
    }

/* linear volume between min_dB and max_dB (.01dB unit) */
#define SNDRV_CTL_TLVD_DB_LINEAR_ITEM(min_dB, max_dB) \
    SNDRV_CTL_TLVD_ITEM(SNDRV_CTL_TLVT_DB_LINEAR, (min_dB), (max_dB))
#define SNDRV_CTL_TLVD_DECLARE_DB_LINEAR(name, min_dB, max_dB) \
    unsigned int name[] = { \
        SNDRV_CTL_TLVD_DB_LINEAR_ITEM(min_dB, max_dB) \
    }

#define DECLARE_TLV_DB_SCALE        SNDRV_CTL_TLVD_DECLARE_DB_SCALE
#define DECLARE_TLV_DB_MINMAX        SNDRV_CTL_TLVD_DECLARE_DB_MINMAX
#define TLV_DB_LINEAR_ITEM        SNDRV_CTL_TLVD_DB_LINEAR_ITEM

#define DECLARE_TLV_DB_LINEAR        SNDRV_CTL_TLVD_DECLARE_DB_LINEAR

#define TLV_DB_GAIN_MUTE        SNDRV_CTL_TLVD_DB_GAIN_MUTE

#define SNDRV_CTL_ELEM_ID_NAME_MAXLEN    44

#define __force 

#define IEC958_AES0_NONAUDIO        (1<<1)    /* 0 = audio, 1 = non-audio */
#define IEC958_AES0_CON_NOT_COPYRIGHT    (1<<2)    /* 0 = copyright, 1 = not copyright */
#define IEC958_AES0_CON_EMPHASIS    (7<<3)    /* mask - emphasis */

#define IEC958_AES1_CON_CATEGORY    0x7f
#define IEC958_AES1_CON_ORIGINAL    (1<<7)    /* this bits depends on the category code */

#define IEC958_AES3_CON_FS        (15<<0)    /* mask - sample frequency */
#define IEC958_AES3_CON_FS_44100    (0<<0)    /* 44.1kHz */
#define IEC958_AES3_CON_FS_NOTID    (1<<0)    /* non indicated */
#define IEC958_AES3_CON_FS_48000    (2<<0)    /* 48kHz */
#define IEC958_AES3_CON_FS_32000    (3<<0)    /* 32kHz */
#define IEC958_AES3_CON_FS_22050    (4<<0)    /* 22.05kHz */
#define IEC958_AES3_CON_FS_24000    (6<<0)    /* 24kHz */
#define IEC958_AES3_CON_FS_88200    (8<<0)    /* 88.2kHz */
#define IEC958_AES3_CON_FS_768000    (9<<0)    /* 768kHz */
#define IEC958_AES3_CON_FS_96000    (10<<0)    /* 96kHz */
#define IEC958_AES3_CON_FS_176400    (12<<0)    /* 176.4kHz */
#define IEC958_AES3_CON_FS_192000    (14<<0)    /* 192kHz */



#define IEC958_AES1_CON_DIGDIGCONV_ID   0x02
#define IEC958_AES1_CON_PCM_CODER       (IEC958_AES1_CON_DIGDIGCONV_ID|0x00)
#define IEC958_AES3_CON_FS_48000    (2<<0)    /* 48kHz */
#define IEC958_AES3_CON_FS_44100        (0<<0)  /* 44.1kHz */
#define IEC958_AES3_CON_FS_96000        (10<<0) /* 96kHz */
#define IEC958_AES3_CON_FS_192000       (14<<0) /* 192kHz */

struct list_head {
    struct list_head *next, *prev;
};
typedef int snd_ctl_elem_type_t;
#define    SNDRV_CTL_ELEM_TYPE_NONE    ((__force snd_ctl_elem_type_t) 0) /* invalid */
#define    SNDRV_CTL_ELEM_TYPE_BOOLEAN    ((__force snd_ctl_elem_type_t) 1) /* boolean type */
#define    SNDRV_CTL_ELEM_TYPE_INTEGER    ((__force snd_ctl_elem_type_t) 2) /* integer type */
#define    SNDRV_CTL_ELEM_TYPE_ENUMERATED    ((__force snd_ctl_elem_type_t) 3) /* enumerated type */
#define    SNDRV_CTL_ELEM_TYPE_BYTES    ((__force snd_ctl_elem_type_t) 4) /* byte array */
#define    SNDRV_CTL_ELEM_TYPE_IEC958    ((__force snd_ctl_elem_type_t) 5) /* IEC958 (S/PDIF) setup */
#define    SNDRV_CTL_ELEM_TYPE_INTEGER64    ((__force snd_ctl_elem_type_t) 6) /* 64-bit integer type */
#define    SNDRV_CTL_ELEM_TYPE_LAST    SNDRV_CTL_ELEM_TYPE_INTEGER64

typedef int snd_ctl_elem_iface_t;
#define    SNDRV_CTL_ELEM_IFACE_CARD    ((__force snd_ctl_elem_iface_t) 0) /* global control */
#define    SNDRV_CTL_ELEM_IFACE_HWDEP    ((__force snd_ctl_elem_iface_t) 1) /* hardware dependent device */
#define    SNDRV_CTL_ELEM_IFACE_MIXER    ((__force snd_ctl_elem_iface_t) 2) /* virtual mixer device */
#define    SNDRV_CTL_ELEM_IFACE_PCM    ((__force snd_ctl_elem_iface_t) 3) /* PCM device */
#define    SNDRV_CTL_ELEM_IFACE_RAWMIDI    ((__force snd_ctl_elem_iface_t) 4) /* RawMidi device */
#define    SNDRV_CTL_ELEM_IFACE_TIMER    ((__force snd_ctl_elem_iface_t) 5) /* timer device */
#define    SNDRV_CTL_ELEM_IFACE_SEQUENCER    ((__force snd_ctl_elem_iface_t) 6) /* sequencer client */
#define    SNDRV_CTL_ELEM_IFACE_LAST    SNDRV_CTL_ELEM_IFACE_SEQUENCER

#define SNDRV_CTL_ELEM_ACCESS_READ        (1<<0)
#define SNDRV_CTL_ELEM_ACCESS_WRITE        (1<<1)
#define SNDRV_CTL_ELEM_ACCESS_READWRITE        (SNDRV_CTL_ELEM_ACCESS_READ|SNDRV_CTL_ELEM_ACCESS_WRITE)
#define SNDRV_CTL_ELEM_ACCESS_VOLATILE        (1<<2)    /* control value may be changed without a notification */
#define SNDRV_CTL_ELEM_ACCESS_TIMESTAMP        (1<<3)    /* when was control changed */
#define SNDRV_CTL_ELEM_ACCESS_TLV_READ        (1<<4)    /* TLV read is possible */
#define SNDRV_CTL_ELEM_ACCESS_TLV_WRITE        (1<<5)    /* TLV write is possible */
#define SNDRV_CTL_ELEM_ACCESS_TLV_READWRITE    (SNDRV_CTL_ELEM_ACCESS_TLV_READ|SNDRV_CTL_ELEM_ACCESS_TLV_WRITE)
#define SNDRV_CTL_ELEM_ACCESS_TLV_COMMAND    (1<<6)    /* TLV command is possible */
#define SNDRV_CTL_ELEM_ACCESS_INACTIVE        (1<<8)    /* control does actually nothing, but may be updated */
#define SNDRV_CTL_ELEM_ACCESS_LOCK        (1<<9)    /* write lock */
#define SNDRV_CTL_ELEM_ACCESS_OWNER        (1<<10)    /* write lock owner */
#define SNDRV_CTL_ELEM_ACCESS_TLV_CALLBACK    (1<<28)    /* kernel use a TLV callback */
#define SNDRV_CTL_ELEM_ACCESS_USER        (1<<29) /* user space element */



#define SNDRV_CTL_NAME_NONE                ""
#define SNDRV_CTL_NAME_PLAYBACK                "Playback "
#define SNDRV_CTL_NAME_CAPTURE                "Capture "

#define SNDRV_CTL_NAME_IEC958_NONE            ""
#define SNDRV_CTL_NAME_IEC958_SWITCH            "Switch"
#define SNDRV_CTL_NAME_IEC958_VOLUME            "Volume"
#define SNDRV_CTL_NAME_IEC958_DEFAULT            "Default"
#define SNDRV_CTL_NAME_IEC958_MASK            "Mask"
#define SNDRV_CTL_NAME_IEC958_CON_MASK            "Con Mask"
#define SNDRV_CTL_NAME_IEC958_PRO_MASK            "Pro Mask"
#define SNDRV_CTL_NAME_IEC958_PCM_STREAM        "PCM Stream"
#define SNDRV_CTL_NAME_IEC958(expl,direction,what)    "IEC958 " expl SNDRV_CTL_NAME_##direction SNDRV_CTL_NAME_IEC958_##what

typedef int        __kernel_pid_t;

struct seq_file {
    char *buf;
    size_t size;
    size_t from;
    size_t count;
    size_t pad_until;
    off_t index;
    off_t read_pos;
    UInt64 version;
};

/* buffer for information */
struct snd_info_buffer {
    char *buffer;        /* pointer to begin of buffer */
    unsigned int curr;    /* current position in buffer */
    unsigned int size;    /* current size */
    unsigned int len;    /* total length of buffer */
    int stop;        /* stop flag */
    int error;        /* error code */
};


struct snd_ctl_elem_id {
    unsigned int numid;        /* numeric identifier, zero = invalid */
    snd_ctl_elem_iface_t iface;    /* interface identifier */
    unsigned int device;        /* device/client number */
    unsigned int subdevice;        /* subdevice (substream) number */
    char name[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];        /* ASCII name of item */
    unsigned int index;        /* index of item */
};


struct snd_ctl_elem_list {
    unsigned int offset;        /* W: first element ID to get */
    unsigned int space;        /* W: count of element IDs to get */
    unsigned int used;        /* R: count of element IDs set */
    unsigned int count;        /* R: count of all elements */
    struct snd_ctl_elem_id  *pids; /* R: IDs */
    unsigned char reserved[50];
};

struct snd_ctl_elem_info {
    struct snd_ctl_elem_id id;    /* W: element ID */
    snd_ctl_elem_type_t type;    /* R: value type - SNDRV_CTL_ELEM_TYPE_* */
    unsigned int access;        /* R: value access (bitmask) - SNDRV_CTL_ELEM_ACCESS_* */
    unsigned int count;        /* count of values */
    __kernel_pid_t owner;        /* owner's PID of this control */
    union {
        struct {
            long min;        /* R: minimum value */
            long max;        /* R: maximum value */
            long step;        /* R: step (0 variable) */
        } integer;
        struct {
            long long min;        /* R: minimum value */
            long long max;        /* R: maximum value */
            long long step;        /* R: step (0 variable) */
        } integer64;
        struct {
            unsigned int items;    /* R: number of items */
            unsigned int item;    /* W: item number */
            char name[64];        /* R: value name */
            UInt64 names_ptr;    /* W: names list (ELEM_ADD only) */
            unsigned int names_length;
        } enumerated;
        unsigned char reserved[128];
    } value;
    union {
        unsigned short d[4];        /* dimensions */
        unsigned short *d_ptr;        /* indirect - obsoleted */
    } dimen;
    unsigned char reserved[64-4*sizeof(unsigned short)];
};

struct snd_aes_iec958 {
    unsigned char status[24];    /* AES/IEC958 channel status bits */
    unsigned char subcode[147];    /* AES/IEC958 subcode bits */
    unsigned char pad;        /* nothing */
    unsigned char dig_subframe[4];    /* AES/IEC958 subframe bits */
};

struct snd_ctl_elem_value {
    struct snd_ctl_elem_id id;    /* W: element ID */
    unsigned int indirect: 1;    /* W: indirect access - obsoleted */
    union {
        union {
            long value[128];
            long *value_ptr;    /* obsoleted */
        } integer;
        union {
            long long value[64];
            long long *value_ptr;    /* obsoleted */
        } integer64;
        union {
            unsigned int item[128];
            unsigned int *item_ptr;    /* obsoleted */
        } enumerated;
        union {
            unsigned char data[512];
            unsigned char *data_ptr;    /* obsoleted */
        } bytes;
        struct snd_aes_iec958 iec958;
    } value;        /* RO */
    struct timespec tstamp;
    unsigned char reserved[128-sizeof(struct timespec)];
};

struct snd_ctl_tlv {
    unsigned int numid;    /* control element numeric identification */
    unsigned int length;    /* in bytes aligned to 4 */
    unsigned int tlv[0];    /* first TLV */
};


struct snd_kcontrol;
typedef int (snd_kcontrol_info_t) (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_info * uinfo);
typedef int (snd_kcontrol_get_t) (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol);
typedef int (snd_kcontrol_put_t) (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol);
typedef int (snd_kcontrol_tlv_rw_t)(struct snd_kcontrol *kcontrol,
                    int op_flag, /* SNDRV_CTL_TLV_OP_XXX */
                    unsigned int size,
                    unsigned int *tlv);

enum {
    SNDRV_CTL_TLV_OP_READ = 0,
    SNDRV_CTL_TLV_OP_WRITE = 1,
    SNDRV_CTL_TLV_OP_CMD = -1,
};

struct snd_kcontrol_new {
    snd_ctl_elem_iface_t iface;    /* interface identifier */
    unsigned int device;        /* device/client number */
    unsigned int subdevice;        /* subdevice (substream) number */
    const char *name;    /* ASCII name of item */
    unsigned int index;        /* index of item */
    unsigned int access;        /* access rights */
    unsigned int count;        /* count of same elements */
    snd_kcontrol_info_t *info;
    snd_kcontrol_get_t *get;
    snd_kcontrol_put_t *put;
    union {
        snd_kcontrol_tlv_rw_t *c;
        const unsigned int *p;
    } tlv;
    unsigned long private_value;
};

struct snd_kcontrol_volatile {
    struct snd_ctl_file *owner;    /* locked */
    unsigned int access;    /* access rights */
};

struct snd_kcontrol {
    struct list_head list;        /* list of controls */
    struct snd_ctl_elem_id id;
    unsigned int count;        /* count of same elements */
    snd_kcontrol_info_t *info;
    snd_kcontrol_get_t *get;
    snd_kcontrol_put_t *put;
    union {
        snd_kcontrol_tlv_rw_t *c;
        const unsigned int *p;
    } tlv;
    unsigned long private_value;
    void *private_data;
    void (*private_free)(struct snd_kcontrol *kcontrol);
    struct snd_kcontrol_volatile vd[0];    /* volatile data */
};

#define snd_kcontrol(n) list_entry(n, struct snd_kcontrol, list)
inline int snd_ctl_enum_info(struct snd_ctl_elem_info *info, unsigned int channels,
              unsigned int items, const char *const names[])
{
    info->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
    info->count = channels;
    info->value.enumerated.items = items;
    if (!items)
        return 0;
    if (info->value.enumerated.item >= items)
        info->value.enumerated.item = items - 1;
    if(strlen(names[info->value.enumerated.item]) >= sizeof(info->value.enumerated.name))
        kprintf("ALSA: too long item name '%s'\n",
         names[info->value.enumerated.item]);
    strlcpy(info->value.enumerated.name,
        names[info->value.enumerated.item],
        sizeof(info->value.enumerated.name));
    return 0;
}

#define snd_iprintf(buf, fmt, args...) \
seq_printf((struct seq_file *)(buf)->buffer, fmt, ##args)



inline void seq_set_overflow(struct seq_file *m)
{
    m->count = m->size;
}


inline void seq_vprintf(struct seq_file *m, const char *f, va_list args)
{
    int len;

    if (m->count < m->size) {
        len = vsnprintf(m->buf + m->count, m->size - m->count, f, args);
        if (m->count + len < m->size) {
            m->count += len;
            return;
        }
    }
    seq_set_overflow(m);
}
inline void seq_printf(struct seq_file *m, const char *f, ...)
{
    va_list args;

    va_start(args, f);
    seq_vprintf(m, f, args);
    va_end(args);
}


inline int snd_ctl_boolean_mono_info(struct snd_kcontrol *kcontrol,
                  struct snd_ctl_elem_info *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}

#endif /* alsa_h */
