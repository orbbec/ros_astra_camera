#ifndef ASTRA_DEVICE_TYPE_H
#define ASTRA_DEVICE_TYPE_H

#define OB_STEREO_S "Orbbec Canglong"
#define OB_EMBEDDED_S "Astra SL1000S_U3"
#define OB_STEREO_S_U3 "Astra SV1301S_U3"
#define OB_ASTRA_PRO "Orbbec Astra Pro"
#define OB_ASTRA_PRO_PLUS "Orbbec Astra Pro Plus"
#define OB_DABAI "Orbbec Dabai"
#define OB_ASTRA_PLUS "Orbbec Astra+"
#define OB_ASTRA_PLUS_S "Orbbec Astra+_S"

#define OB_DABAI_PRO "Orbbec Astra DaBai Pro"
#define OB_DABAI_DCW "Orbbec DaBai DCW"
#define OB_DABAI_DW "Orbbec DaBai DW"

typedef enum {
  OB_ASTRA_NO,
  OB_STEREO_S_NO,
  OB_EMBEDDED_S_NO,
  OB_STEREO_S_U3_NO,
  OB_ASTRA_PRO_NO,
  OB_ASTRA_PRO_PLUS_NO,
  OB_DABAI_NO,
  OB_ASTRA_PLUS_NO,
  OB_ASTRA_PLUS_S_NO,
  OB_DABAI_PRO_NO,
  OB_DABAI_DCW_NO,
  OB_DABAI_DW_NO
} OB_DEVICE_NO;

typedef enum {
  RGBResolution4_3 = 0,   // 4:3分辨率 如：640x480
  RGBResolution16_9 = 1,  // 16:9分辨率 如：1920x1280}RgbResolution;
} RgbResolution;

bool astraWithUVC(OB_DEVICE_NO id);

#endif
