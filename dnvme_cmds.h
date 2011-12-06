/*
 * NVM Express Compliance Suite
 * Copyright (c) 2011, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _DNVME_CMDS_H_
#define _DNVME_CMDS_H_

/* define's for unique QID creation */
#define UNIQUE_QID_FLAG 0x01

/* Enum specifying Writes/Reads to mapped pages and other general enums */
enum {
    READ_PG = 0,
    WRITE_PG = 1,
    PRP_PRESENT = 1, /* Specifies to geberate PRP's for a particular command */
    PRP_ABSENT = 0, /* Specifies not to generate PRP's per command */
    PRP_Size = 8, /* Size of PRP entry in bytes */
    PERSIST_QID_0 = 0, /* Default value of Persist queue ID */
    CDW11_PC = 1, /* Mask for checking CDW11.PC of create IO Q cmds */
    UNIQUE_ID = 2, /* Offset for unique ID within the command */
};

/* Enum specifying PRP1,PRP2 or List */
enum prp_type {
    NO_PRP = 0,
    PRP1 = 1,
    PRP2 = 2,
    PRP_List = 4,
};

/* Enum specifying type of data buffer */
enum data_buf_type {
    DATA_BUF = 0,
    CONTG_IO_Q = 0,
    DISCONTG_IO_Q = 1,
};

/**
 * prep_send64b_cmd:
 * Prepares the 64 byte command to be sent
 * with PRP generation and addition of nodes
 * inside cmd track list
 * @param nvme_dev
 * @param pmetrics_sq
 * @param nvme_64b_send
 * @param prps
 * @param nvme_gen_cmd
 * @param persist_q_id
 * @param data_buf_type
 * @param gen_prp
 * @return Error Codes
 */
int prep_send64b_cmd(struct nvme_device *nvme_dev, struct metrics_sq
    *pmetrics_sq, struct nvme_64b_send *nvme_64b_send, struct nvme_prps *prps,
        struct nvme_gen_cmd *nvme_gen_cmd, __u16 persist_q_id,
            enum data_buf_type data_buf_type, __u8 gen_prp);

/**
 * add_cmd_track_node:
 * Add node inside the cmd track list
 * @param pmetrics_sq
 * @param persist_q_id
 * @param prps
 * @param cmd_type
 * @param opcode
 * @param cmd_id
 * @return Error codes
 */
int add_cmd_track_node(struct  metrics_sq  *pmetrics_sq,
    __u16 persist_q_id, struct nvme_prps *prps, enum nvme_cmds cmd_type,
        __u8 opcode, __u16 cmd_id);

/**
 * empty_cmd_track_list:
 * Delete command track list completley per SQ
 * @param nvme_device
 * @param pmetrics_sq
 * @return void
 */
void empty_cmd_track_list(struct  nvme_device *nvme_device,
    struct  metrics_sq *pmetrics_sq);

/**
 * destroy_dma_pool:
 * Destroy's the dma pool
 * @param nvme_dev
 * @return void
 */
void destroy_dma_pool(struct nvme_device *nvme_dev);

/**
 * del_prps:
 * Deletes the PRP structures of SQ/CQ or command track node
 * @param nvme_device
 * @param prps
 * @return void
 */
void del_prps(struct nvme_device *nvme_device, struct nvme_prps *prps);

#endif
