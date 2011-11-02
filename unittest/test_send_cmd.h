/**
 * Specific structure for NVME Identify command
 */
struct nvme_identify {
    uint8_t  opcode;
    uint8_t  flags;
    uint16_t command_id;
    uint32_t nsid;
    uint64_t rsvd2[2];
    uint64_t prp1;
    uint64_t prp2;
    uint32_t cns;
    uint32_t rsvd11[5];
};

/**
 * Specific structure for NVME IO Read/Write command
 */
struct nvme_user_io {
    uint8_t  opcode;
    uint8_t  flags;
    uint16_t control;
    uint32_t nsid;
    uint64_t rsvd2[1];
    uint64_t metadata;
    uint64_t prp1;
    uint64_t prp2;
    uint64_t slba;
    uint16_t nlb;
    uint16_t cmd_flags;
    uint32_t dsm;
    uint32_t ilbrt;
    uint16_t lbat;
    uint16_t lbatm;
};

