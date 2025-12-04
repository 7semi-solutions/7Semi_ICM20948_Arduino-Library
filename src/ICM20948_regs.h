#ifndef ICM20948_REGS_H
#define ICM20948_REGS_H

/**
 * 7Semi comment style
 * - Full ICM-20948 register map (Banks 0–3) + key bit fields
 * - Simple, portable #defines for firmware use
 * - Datasheet naming kept where practical; fields grouped by bank
 *
 * Notes
 * - WHO_AM_I expected value: 0xEA
 * - Select register bank via REG_BANK_SEL in any bank
 * - Use BANK(n) helper or write raw values 0x00/0x10/0x20/0x30
 */

/* -------- Common helpers -------- */
#ifndef BIT
#define BIT(n) (1u << (n))
#endif

// #define BANK(n) ((uint8_t)((n) << 4))
// #define BANK0 BANK(0)
// #define BANK1 BANK(1)
// #define BANK2 BANK(2)
// #define BANK3 BANK(3)

/* ========================================================================== */
/*                                BANK 0                                      */
/* ========================================================================== */

/** - Identity / power / interrupts / sensor data */
#define WHO_AM_I 0x00 /* WHO_AM_I[7:0] */
#define WHO_AM_I_VAL 0xEA

#define USER_CTRL 0x03 /* DMP_EN FIFO_EN I2C_MST_EN I2C_IF_DIS DMP_RST SRAM_RST I2C_MST_RST - */
/* bits */
#define USER_CTRL_DMP_EN BIT(7)
#define USER_CTRL_FIFO_EN BIT(6)
#define USER_CTRL_I2C_MST_EN BIT(5)
#define USER_CTRL_I2C_IF_DIS BIT(4)
#define USER_CTRL_DMP_RST BIT(3)
#define USER_CTRL_SRAM_RST BIT(2)
#define USER_CTRL_I2C_MST_RST BIT(1)

#define LP_CONFIG 0x05 /* I2C_MST_CYCLE ACCEL_CYCLE GYRO_CYCLE - */
#define LP_I2C_MST_CYCLE BIT(6)
#define LP_ACCEL_CYCLE BIT(5)
#define LP_GYRO_CYCLE BIT(4)

#define PWR_MGMT_1 0x06 /* DEVICE_RESET SLEEP LP_EN - TEMP_DIS CLKSEL[2:0] */
#define PWR_DEVICE_RESET BIT(7)
#define PWR_SLEEP BIT(6)
#define PWR_LP_EN BIT(5)
#define PWR_TEMP_DIS BIT(3)
#define PWR_CLKSEL_MASK 0x07
#define PWR_CLKSEL_INT_20MHZ 0x01
#define PWR_CLKSEL_AUTO 0x01 /* typical: auto selects best source */

#define PWR_MGMT_2 0x07 /* - DISABLE_ACCEL DISABLE_GYRO */
#define PWR_DISABLE_ACCEL BIT(3)
#define PWR_DISABLE_GYRO BIT(0)

#define INT_PIN_CFG 0x0F         /* INT1_ACTL INT1_OPEN INT1_LATCH_INT_EN INT_ANYRD_2CLEAR ACTL_FSYNC FSYNC_INT_MODE_EN BYPASS_EN - */
#define INT1_ACTL BIT(7)         /* active low */
#define INT1_OPEN BIT(6)         /* open-drain */
#define INT1_LATCH_INT_EN BIT(5) /* latch until status read */
#define INT_ANYRD_2CLEAR BIT(4)
#define INT_ACTL_FSYNC BIT(3)
#define FSYNC_INT_MODE_EN BIT(2)
#define BYPASS_EN BIT(1) /* I2C bypass to aux devices */

#define INT_ENABLE 0x10 /* REG_WOF_EN - WOM_INT_EN PLL_RDY_EN DMP_INT1_EN I2C_MST_INT_EN */
#define INT_REG_WOF_EN BIT(7)
#define INT_WOM_INT_EN BIT(3)
#define INT_PLL_RDY_EN BIT(2)
#define INT_DMP_INT1_EN BIT(1)
#define INT_I2C_MST_INT_EN BIT(0)

#define INT_ENABLE_1 0x11 /* RAW_DATA_0_RDY_EN */
#define INT_RAW_DATA_0_RDY_EN BIT(0)

#define INT_ENABLE_2 0x12 /* FIFO_OVERFLOW_EN[4:0] */
#define INT_ENABLE_3 0x13 /* FIFO_WM_EN[4:0] */

#define I2C_MST_STATUS 0x17 /* PASS_THROUGH, *_DONE, *_NACK, LOST_ARB */
#define MST_PASS_THROUGH BIT(7)
#define MST_SLV4_DONE BIT(6)
#define MST_LOST_ARB BIT(5)
#define MST_SLV4_NACK BIT(4)
#define MST_SLV3_NACK BIT(3)
#define MST_SLV2_NACK BIT(2)
#define MST_SLV1_NACK BIT(1)
#define MST_SLV0_NACK BIT(0)

#define INT_STATUS 0x19 /* WOM_INT PLL_RDY_INT DMP_INT1 I2C_MST_INT */
#define STS_WOM_INT BIT(3)
#define STS_PLL_RDY_INT BIT(2)
#define STS_DMP_INT1 BIT(1)
#define STS_I2C_MST_INT BIT(0)

#define INT_STATUS_1 0x1A /* RAW_DATA_0_RDY_INT */
#define STS_RAW_DATA_0_RDY_INT BIT(0)
#define INT_STATUS_2 0x1B /* FIFO_OVERFLOW_INT[4:0] */
#define INT_STATUS_3 0x1C /* FIFO_WM_INT[4:0] */

#define DELAY_TIMEH 0x28
#define DELAY_TIMEL 0x29

/* - Sensor outputs */
#define ACCEL_XOUT_H 0x2D
#define ACCEL_XOUT_L 0x2E
#define ACCEL_YOUT_H 0x2F
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32
#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38
#define TEMP_OUT_H 0x39
#define TEMP_OUT_L 0x3A

/* - External sensor shadow data (aux I2C) */
#define EXT_SLV_SENS_DATA_00 0x3B
#define EXT_SLV_SENS_DATA_23 0x52 /* contiguous range 0x3B..0x52 */

#define FIFO_EN_1 0x66 /* SLV3..SLV0 FIFO_EN */
#define FIFO_EN_2 0x67 /* ACCEL GYRO_Z/Y/X TEMP FIFO_EN */
#define FIFO_RST 0x68  /* FIFO_RESET[4:0] */
#define FIFO_MODE 0x69 /* FIFO_MODE[4:0] */
#define FIFO_COUNTH 0x70
#define FIFO_COUNTL 0x71
#define FIFO_R_W 0x72
// #define DATA_RDY_STATUS 0x74 /* WOF_STATUS RAW_DATA_RDY[3:0] */
#define FIFO_CFG 0x76

#define REG_BANK_SEL 0x7F /* USER_BANK[1:0] */

/* ========================================================================== */
/*                                BANK 1                                      */
/* ========================================================================== */

/** - Self-test / accel offsets / timebase */
#define SELF_TEST_X_GYRO 0x02  /* XG_ST_DATA[7:0] */
#define SELF_TEST_Y_GYRO 0x03  /* YG_ST_DATA[7:0] */
#define SELF_TEST_Z_GYRO 0x04  /* ZG_ST_DATA[7:0] */
#define SELF_TEST_X_ACCEL 0x0E /* XA_ST_DATA[7:0] */
#define SELF_TEST_Y_ACCEL 0x0F /* YA_ST_DATA[7:0] */
#define SELF_TEST_Z_ACCEL 0x10 /* ZA_ST_DATA[7:0] */

#define XA_OFFS_H 0x14 /* XA_OFFS[14:7] */
#define XA_OFFS_L 0x15 /* XA_OFFS[6:0] */
#define YA_OFFS_H 0x17 /* YA_OFFS[14:7] */
#define YA_OFFS_L 0x18 /* YA_OFFS[6:0] */
#define ZA_OFFS_H 0x1A /* ZA_OFFS[14:7] */
#define ZA_OFFS_L 0x1B /* ZA_OFFS[6:0] */

#define TIMEBASE_CORRECTION_PLL 0x28 /* TBC_PLL[7:0] */

#define BANK1_REG_BANK_SEL 0x7F /* mirror of REG_BANK_SEL */

/* ========================================================================== */
/*                                BANK 2                                      */
/* ========================================================================== */

/** - Gyro/Accel configuration, offsets, FSYNC, temperature filter */
#define GYRO_SMPLRT_DIV 0x00 /* GYRO_SMPLRT_DIV[7:0] */

#define GYRO_CONFIG_1 0x01  /* GYRO_DLPFCFG[2:0] GYRO_FS_SEL[1:0] GYRO_FCHOICE */
#define GYRO_FCHOICE BIT(0) /* when 0: DLPF on, when 1: off (per datasheet) */
#define GYRO_FS_SEL_SHIFT 1
#define GYRO_FS_SEL_MASK (0x3u << GYRO_FS_SEL_SHIFT)
#define GYRO_FS_250DPS (0u << GYRO_FS_SEL_SHIFT)
#define GYRO_FS_500DPS (1u << GYRO_FS_SEL_SHIFT)
#define GYRO_FS_1000DPS (2u << GYRO_FS_SEL_SHIFT)
#define GYRO_FS_2000DPS (3u << GYRO_FS_SEL_SHIFT)
#define GYRO_DLPFCFG_SHIFT 5
#define GYRO_DLPFCFG_MASK (0x7u << GYRO_DLPFCFG_SHIFT)

#define GYRO_CONFIG_2 0x02 /* XGYRO_CTEN YGYRO_CTEN ZGYRO_CTEN GYRO_AVGCFG[2:0] */
#define XGYRO_CTEN BIT(5)
#define YGYRO_CTEN BIT(4)
#define ZGYRO_CTEN BIT(3)
#define GYRO_AVGCFG_SHIFT 0
#define GYRO_AVGCFG_MASK (0x7u << GYRO_AVGCFG_SHIFT)

#define XG_OFFS_USRH 0x03
#define XG_OFFS_USRL 0x04
#define YG_OFFS_USRH 0x05
#define YG_OFFS_USRL 0x06
#define ZG_OFFS_USRH 0x07
#define ZG_OFFS_USRL 0x08

#define ODR_ALIGN_EN 0x09 /* ODR_ALIGN_EN */
#define ODR_ALIGN_EN_BIT BIT(0)

#define ACCEL_SMPLRT_DIV_1 0x10 /* ACCEL_SMPLRT_DIV[11:8] */
#define ACCEL_SMPLRT_DIV_2 0x11 /* ACCEL_SMPLRT_DIV[7:0] */

#define ACCEL_INTEL_CTRL 0x12 /* ACCEL_INTEL_EN ACCEL_INTEL_MODE_INT */
#define ACCEL_INTEL_EN BIT(7)
#define ACCEL_INTEL_MODE_INT BIT(6)

#define ACCEL_WOM_THR 0x13 /* WOM_THRESHOLD[7:0] */

#define ACCEL_CONFIG 0x14 /* ACCEL_DLPFCFG[2:0] ACCEL_FS_SEL[1:0] ACCEL_FCHOICE */
#define ACCEL_FCHOICE BIT(0)
#define ACCEL_FS_SEL_SHIFT 1
#define ACCEL_FS_SEL_MASK (0x3u << ACCEL_FS_SEL_SHIFT)
#define ACCEL_FS_2G (0u << ACCEL_FS_SEL_SHIFT)
#define ACCEL_FS_4G (1u << ACCEL_FS_SEL_SHIFT)
#define ACCEL_FS_8G (2u << ACCEL_FS_SEL_SHIFT)
#define ACCEL_FS_16G (3u << ACCEL_FS_SEL_SHIFT)
#define ACCEL_DLPFCFG_SHIFT 5
#define ACCEL_DLPFCFG_MASK (0x7u << ACCEL_DLPFCFG_SHIFT)

#define ACCEL_CONFIG_2 0x15 /* AX_ST_EN_REG AY_ST_EN_REG AZ_ST_EN_REG DEC3_CFG[1:0] */
#define AX_ST_EN_REG BIT(7)
#define AY_ST_EN_REG BIT(6)
#define AZ_ST_EN_REG BIT(5)
#define DEC3_CFG_SHIFT 0
#define DEC3_CFG_MASK (0x3u << DEC3_CFG_SHIFT)

#define FSYNC_CONFIG 0x52 /* DELAY_TIME_EN WOF_DEGLITCH_EN WOF_EDGE_INT EXT_SYNC_SET[3:0] */
#define DELAY_TIME_EN BIT(7)
#define WOF_DEGLITCH_EN BIT(6)
#define WOF_EDGE_INT BIT(5)
#define EXT_SYNC_SET_MASK 0x0F

#define TEMP_CONFIG 0x53 /* TEMP_DLPFCFG[2:0] */
#define TEMP_DLPFCFG_SHIFT 5
#define TEMP_DLPFCFG_MASK (0x7u << TEMP_DLPFCFG_SHIFT)

#define MOD_CTRL_USR 0x54 /* REG_LP_DMP_EN */
#define REG_LP_DMP_EN BIT(7)

#define BANK2_REG_BANK_SEL 0x7F /* mirror of REG_BANK_SEL */

/* ========================================================================== */
/*                                BANK 3                                      */
/* ========================================================================== */

/** - I2C master (aux) interface and slave windows */
#define I2C_MST_ODR_CONFIG 0x00 /* I2C_MST_ODR_CONFIG[3:0] */
#define MST_ODR_CFG_MASK 0x0F

#define I2C_MST_CTRL 0x01 /* MULT_MST_EN - I2C_MST_P_NSR I2C_MST_CLK[3:0] */
#define MULT_MST_EN BIT(7)
#define I2C_MST_P_NSR BIT(4)
#define I2C_MST_CLK_MASK 0x0F

#define I2C_MST_DELAY_CTRL 0x02 /* DELAY_ES_SHADOW + I2C_SLVx_DELAY_EN bits */
#define DELAY_ES_SHADOW BIT(7)
#define I2C_SLV4_DELAY_EN BIT(4)
#define I2C_SLV3_DELAY_EN BIT(3)
#define I2C_SLV2_DELAY_EN BIT(2)
#define I2C_SLV1_DELAY_EN BIT(1)
#define I2C_SLV0_DELAY_EN BIT(0)

#define I2C_SLV0_ADDR 0x03 /* RNW + ID[6:0] */
#define I2C_SLVx_RNW BIT(7)
#define I2C_SLV0_REG 0x04
#define I2C_SLV0_CTRL 0x05 /* EN BYTE_SW REG_DIS GRP LENG[3:0] */
#define I2C_SLVx_EN BIT(7)
#define I2C_SLVx_BYTE_SW BIT(6)
#define I2C_SLVx_REG_DIS BIT(5)
#define I2C_SLVx_GRP BIT(4)
#define I2C_SLVx_LENG_MASK 0x0F
#define I2C_SLV0_DO 0x06

#define I2C_SLV1_ADDR 0x07
#define I2C_SLV1_REG 0x08
#define I2C_SLV1_CTRL 0x09
#define I2C_SLV1_DO 0x0A

#define I2C_SLV2_ADDR 0x0B
#define I2C_SLV2_REG 0x0C
#define I2C_SLV2_CTRL 0x0D
#define I2C_SLV2_DO 0x0E

#define I2C_SLV3_ADDR 0x0F
#define I2C_SLV3_REG 0x10
#define I2C_SLV3_CTRL 0x11
#define I2C_SLV3_DO 0x12

#define I2C_SLV4_ADDR 0x13
#define I2C_SLV4_REG 0x14
#define I2C_SLV4_CTRL 0x15 /* EN BYTE_SW REG_DIS DLY[4:0] */
#define I2C_SLV4_DLY_MASK 0x1F
#define I2C_SLV4_DO 0x16
#define I2C_SLV4_DI 0x17

#define BANK3_REG_BANK_SEL 0x7F /* mirror of REG_BANK_SEL */

/* ========================================================================== */
/*                         REG_BANK_SEL (all banks)                           */
/* ========================================================================== */

#define REG_BANK_SEL_USER_BANK_SHIFT 4
#define REG_BANK_SEL_USER_BANK_MASK (0x3u << REG_BANK_SEL_USER_BANK_SHIFT)
#define USER_BANK_0 BANK0
#define USER_BANK_1 BANK1
#define USER_BANK_2 BANK2
#define USER_BANK_3 BANK3

/* AK09916 magnetometer registers (connected internally) */
#define AK09916_I2C_ADDR 0x0C
#define AK_WIA2 0x01
#define AK_ST1 0x10
#define AK_HXL 0x11
#define AK_ST2 0x18
#define AK_CNTL2 0x31
#define AK_CNTL3 0x32
#define AK_WIA2_VAL 0x09 /* AK09916C WIA2 expected */

#define INTERNAL_20MHZ 0x00 /* 0: Internal 20 MHz RC */
#define AUTO_SEL 0x01       /* 1–5: Auto/PLL preferred (use 1 as default) */
#define CLK_STOP 0x07       /* 7: Stop clock / timing gen reset */


/* ------------------ Accelerometer Range ------------------ */
/** - ±2g / ±4g / ±8g / ±16g (maps to FS_SEL 0..3) */
#define g2    0
#define g4    1
#define g8    2
#define g16   3

/* ------------- Accelerometer Filter Path (FCHOICE) ------------- */
/**
 * Path select for accel:
 * - ACCEL_FCHOICE_BYPASS : DLPF bypassed (very wide BW, fastest)
 * - ACCEL_FCHOICE_DLPF   : DLPF enabled (use ACCEL_DLPFCFG + SMPLRT_DIV)
 */
#define ACCEL_FCHOICE_BYPASS   0
#define ACCEL_FCHOICE_DLPF     1

/* --------------- Accelerometer DLPF Config (CFG) --------------- */
/**
 * ACCEL_DLPFCFG (0..7) — choose cutoff/noise BW set (when DLPF is ON).
 * Tip: start with ACCEL_DLPFCFG_3 for a good noise/latency tradeoff.
 */
#define ACCEL_DLPFCFG_0        0
#define ACCEL_DLPFCFG_1        1
#define ACCEL_DLPFCFG_2        2
#define ACCEL_DLPFCFG_3        3
#define ACCEL_DLPFCFG_4        4
#define ACCEL_DLPFCFG_5        5
#define ACCEL_DLPFCFG_6        6
#define ACCEL_DLPFCFG_7        7

#define ACCEL_DEC3_AVG_4       0u  /* averages 1 or 4 (depends on FCHOICE) */
#define ACCEL_DEC3_AVG_8       1u
#define ACCEL_DEC3_AVG_16      2u
#define ACCEL_DEC3_AVG_32      3u

/* ---- Accel DLPF ODR helper (Hz) : base 1125 Hz, DIV 0..4095 ---- */
#define ACCEL_SMPLRT_DIV_MIN     0u
#define ACCEL_SMPLRT_DIV_MAX     4095u
#define ACCEL_DLPF_BASE_HZ       1125.0f
#define ACCEL_DLPF_ODR_HZ(div)   (ACCEL_DLPF_BASE_HZ / (1.0f + (float)(div)))

/* ---- Accel BYPASS quick reference (FCHOICE=0) ----
 * 3dB ≈ 1209 Hz, NBW ≈ 1248 Hz, output rate ≈ 4500 Hz
 * (No divider/ODR control in bypass path.)
 */
#define ACCEL_BYPASS_3DB_BW_HZ   1209.0f
#define ACCEL_BYPASS_NBW_HZ      1248.0f
#define ACCEL_BYPASS_RATE_HZ     4500.0f

/* (Optional) Accel DLPF nominal bandwidth references (FCHOICE=1)
 * Keep here for quick docs; exact values depend on datasheet rev.
 */
#define ACCEL_DLPF0_3DB_BW_HZ    246.0f
#define ACCEL_DLPF0_NBW_HZ       265.0f
#define ACCEL_DLPF1_3DB_BW_HZ    246.0f
#define ACCEL_DLPF1_NBW_HZ       265.0f
#define ACCEL_DLPF2_3DB_BW_HZ    111.4f
#define ACCEL_DLPF2_NBW_HZ       136.0f
#define ACCEL_DLPF3_3DB_BW_HZ    50.4f
#define ACCEL_DLPF3_NBW_HZ       68.8f
#define ACCEL_DLPF4_3DB_BW_HZ    23.9f
#define ACCEL_DLPF4_NBW_HZ       34.4f
#define ACCEL_DLPF5_3DB_BW_HZ    11.5f
#define ACCEL_DLPF5_NBW_HZ       17.0f
#define ACCEL_DLPF6_3DB_BW_HZ    5.7f
#define ACCEL_DLPF6_NBW_HZ       8.3f
#define ACCEL_DLPF7_3DB_BW_HZ    473.0f
#define ACCEL_DLPF7_NBW_HZ       499.0f


/* -------------------- Gyroscope Range -------------------- */
/** - ±250/±500/±1000/±2000 dps (maps to FS_SEL 0..3) */
#define dps250   0
#define dps500   1
#define dps1000  2
#define dps2000  3

/* --------------- Gyroscope Filter Path (FCHOICE) --------------- */
/**
 * Path select for gyro:
 * - GYRO_FCHOICE_BYPASS : DLPF bypassed (widest BW, fastest)
 * - GYRO_FCHOICE_DLPF   : DLPF enabled (use GYRO_DLPFCFG + SMPLRT_DIV)
 */
#define GYRO_FCHOICE_BYPASS   0
#define GYRO_FCHOICE_DLPF     1

/* ----------------- Gyroscope DLPF Config (CFG) ----------------- */
/**
 * GYRO_DLPFCFG (0..7) — choose cutoff/noise BW set (when DLPF is ON).
 * Tip: start with GYRO_DLPFCFG_3 or _2 for balanced noise/latency.
 */
#define GYRO_DLPFCFG_0        0
#define GYRO_DLPFCFG_1        1
#define GYRO_DLPFCFG_2        2
#define GYRO_DLPFCFG_3        3
#define GYRO_DLPFCFG_4        4
#define GYRO_DLPFCFG_5        5
#define GYRO_DLPFCFG_6        6
#define GYRO_DLPFCFG_7        7

/* ---- Gyro DLPF ODR helper (Hz) : base 1100 Hz, guard ≥~4.3 Hz ---- */
#define GYRO_SMPLRT_MIN_HZ      4.3f
#define GYRO_DLPF_BASE_HZ       1100.0f
#define GYRO_DLPF_ODR_HZ(rate_request)  (rate_request)  /* symbolic; your driver computes DIV = round(1100/rate)-1 */

/* (Optional) Gyro BW “buckets” to tag configs in UI/logs (symbolic)
 * Use these if you want a quick human-readable label for NBW tiers.
 * Exact Hz depend on datasheet table; map per your implementation.
 */
#define GYRO_BW_ULTRA_WIDE   0   /* bypass path */
#define GYRO_BW_WIDE         1   /* e.g., CFG 0/1 */
#define GYRO_BW_MEDIUM       2   /* e.g., CFG 2/3 */
#define GYRO_BW_NARROW       3   /* e.g., CFG 4/5/6/7 */

/* ---------------- Convenience: presets (symbolic) --------------- */
/** - Friendly ODR presets you can map to dividers */
#define Hz2    0
#define Hz6    1
#define Hz8    2
#define Hz10   3
#define Hz15   4
#define Hz20   5
#define Hz25   6
#define Hz30   7

/* ------------------ Power / Fusion Modes ----------------- */
/**
 * Generic power or fusion quality modes (BNO-style).
 * Map to sensor-specific sequences inside your driver.
 */
#define LowPower          0
#define Regular           1
#define EnhancedRegular   2
#define HighAccuracy      3

#endif /* ICM20948_REGS_H */
