#define THUMB 
#define INDEX
// #define MIDDLE
// #define RING
#define PINKY

#define FREQ 100
#define IMU 


#ifdef THUMB
#define THUMB_ADDR {0x72,
#define THUMB_OFFSETS -135.25, -297.42,
#define THUMB_HAPTIC 0.0},
#else
#define THUMB_ADDR 
#define THUMB_OFFSETS
#define THUMB_HAPTIC 
#endif

#ifdef INDEX
#define INDEX_ADDR {0x70,
#define INDEX_OFFSETS -122.34, -159.91,
#define INDEX_HAPTIC 0.0},
#else
#define INDEX_ADDR 
#define INDEX_OFFSETS
#define INDEX_HAPTIC 
#endif

#ifdef MIDDLE
#define MIDDLE_ADDR {0x72,
#define MIDDLE_OFFSETS 0.0, 0.0,
#define MIDDLE_HAPTIC 0.0},
#else
#define MIDDLE_ADDR 
#define MIDDLE_OFFSETS
#define MIDDLE_HAPTIC 
#endif

#ifdef RING
#define RING_ADDR {0x72,
#define RING_OFFSETS 0.0, 0.0,
#define RING_HAPTIC 0.0},
#else
#define RING_ADDR 
#define RING_OFFSETS
#define RING_HAPTIC 
#endif

#ifdef PINKY
#define PINKY_ADDR {0x73,
#define PINKY_OFFSETS 0.0, 0.0,
#define PINKY_HAPTIC 1.0},
#else
#define PINKY_ADDR 
#define PINKY_OFFSETS
#define PINKY_HAPTIC 
#endif

#define FINGER_CONFIG   {\
                    THUMB_ADDR THUMB_OFFSETS THUMB_HAPTIC  \
                    INDEX_ADDR INDEX_OFFSETS INDEX_HAPTIC  \
                    MIDDLE_ADDR MIDDLE_OFFSETS MIDDLE_HAPTIC \
                    RING_ADDR RING_OFFSETS RING_HAPTIC   \
                    PINKY_ADDR PINKY_OFFSETS PINKY_HAPTIC  \
                    }


#define LED_COUNT 1
#define LED_PIN 39
#define LED_ENABLE 38
#define BRIGHTNESS 25