#define THUMB 
#define INDEX
// #define MIDDLE
// #define RING
// #define PINKY

#define FREQ 10


#ifdef THUMB
#define THUMB_ADDR 0x72,
#define THUMB_OFFSETS {-135.25, -297.42},
#else
#define THUMB_ADDR 
#define THUMB_OFFSETS 
#endif

#ifdef INDEX
#define INDEX_ADDR 0x70,
#define INDEX_OFFSETS {-122.34, -159.91},
#else
#define INDEX_ADDR 
#define INDEX_OFFSETS 
#endif

#ifdef MIDDLE
#define MIDDLE_ADDR 0x72,
#define MIDDLE_OFFSETS {0.0, 0.0},
#else
#define MIDDLE_ADDR 
#define MIDDLE_OFFSETS 
#endif

#ifdef RING
#define RING_ADDR 0x72,
#define RING_OFFSETS {0.0, 0.0},
#else
#define RING_ADDR 
#define RING_OFFSETS 
#endif

#ifdef PINKY
#define PINKY_ADDR 0x72,
#define PINKY_OFFSETS {0.0, 0.0},
#else
#define PINKY_ADDR 
#define PINKY_OFFSETS 
#endif

#define ADDR_LIST   {\
                    THUMB_ADDR  \
                    INDEX_ADDR  \
                    MIDDLE_ADDR \
                    RING_ADDR   \
                    PINKY_ADDR  \
                    }

#define OFFSETS_LIST    {\
                        THUMB_OFFSETS   \
                        INDEX_OFFSETS   \
                        MIDDLE_OFFSETS  \
                        RING_OFFSETS    \
                        PINKY_OFFSETS   \
                        }