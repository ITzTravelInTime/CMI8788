//
//  mpu401.h
//  PCIAudioDriver
//
//  Created by Gagan on 2022-12-10.
//  Copyright © 2022 CMedia. All rights reserved.
//

#ifndef mpu401_h
#define mpu401_h

#define MPU401_INFO_INPUT    (1 << 0)    /* input stream */
#define MPU401_INFO_OUTPUT    (1 << 1)    /* output stream */
#define MPU401_INFO_INTEGRATED    (1 << 2)    /* integrated h/w port */
#define MPU401_INFO_MMIO    (1 << 3)    /* MMIO access */
#define MPU401_INFO_TX_IRQ    (1 << 4)    /* independent TX irq */
#define MPU401_INFO_IRQ_HOOK    (1 << 5)    /* mpu401 irq handler is called
                           from driver irq handler */
#endif /* mpu401_h */
