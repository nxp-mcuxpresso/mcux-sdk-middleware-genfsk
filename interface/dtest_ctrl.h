/*
 * Copyright 2019 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef DTEST_CTRL_H_
#define DTEST_CTRL_H_

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Defines & Macros Section
///////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(KW36Z4_SERIES)
#define XCVR_DTEST_CTRL (*((uint32_t *)(XCVR_MISC_BASE+0x10u)))
#define XCVR_CTRL_DTEST_CTRL_DTEST_PAGE_MASK     (0x3Fu)
#elif defined(RADIO_IS_GEN_3P5)
#define XCVR_DTEST_CTRL (*((uint32_t *)(RADIO_CTRL_BASE+0x10u)))
#define XCVR_CTRL_DTEST_CTRL_DTEST_PAGE_MASK     (0x7Fu)
#else
#error "unknown radio platform"
#endif
/*! @name DTEST_CTRL - DIGITAL TEST MUX CONTROL */

#define XCVR_CTRL_DTEST_CTRL_DTEST_PAGE_SHIFT    (0U)
#define XCVR_CTRL_DTEST_CTRL_DTEST_PAGE(x)       (((uint32_t)(((uint32_t)(x)) << XCVR_CTRL_DTEST_CTRL_DTEST_PAGE_SHIFT)) & XCVR_CTRL_DTEST_CTRL_DTEST_PAGE_MASK)
#define XCVR_CTRL_DTEST_CTRL_DTEST_EN_MASK       (0x80U)
#define XCVR_CTRL_DTEST_CTRL_DTEST_EN_SHIFT      (7U)
#define XCVR_CTRL_DTEST_CTRL_DTEST_EN(x)         (((uint32_t)(((uint32_t)(x)) << XCVR_CTRL_DTEST_CTRL_DTEST_EN_SHIFT)) & XCVR_CTRL_DTEST_CTRL_DTEST_EN_MASK)


#if defined(KW36Z4_SERIES)
#define DTEST_AA_MACTH_PAGE (0x2A) // aa_sfd_matched in DTEST13 PAGE 0x2A
#define DTEST_TX_DIG_EN_PAGE (0x0A) // tx_dig_enable in DTEST13
#elif defined(RADIO_IS_GEN_3P5)
#define DTEST_AA_MACTH_PAGE (0x2F) // dtest_mux_aa_fnd is in DTEST11 PAGE 0x2F (PHY_MUX)
#define DTEST_MISC_PAGE     (0x54) // lant_sw_active is in DTEST2 ant_tx_out is in DTEST0 PAGE 0x54 (MISC)
#define DTEST_DMA_PAGE      (0x3)
#define DTEST_TX_DIG_EN_PAGE (0x06) // tx_dig_enable in DTEST13
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Typedef Section
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                Function-like Macros Section
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Extern Constants Section
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Extern Variables Section
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                Function Prototypes Section
///////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

#if defined(__cplusplus)
}
#endif // __cplusplus


#endif /* DTEST_CTRL_H_ */
///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
