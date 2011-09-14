#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/uaccess.h>

#include "definitions.h"
#include "sysdnvme.h"
#include "dnvme_ds.h"

/*
 * This file has functions which retrieves the queue metrics when the user
 * app requests. This mainly implements the IOCTL for GET_Q_METRICS.
 */

/*
 * nvme_get_q_metrics will return the q metrics from the global data
 * structures if the q_id send down matches any q_id for this device.
 * If the Q id does not exist in the list then it returns error.
 * This function also returns error when kernel cannot allocate for
 * at-least one element memory of public_sq or public_cq.
 */
int nvme_get_q_metrics1(struct nvme_get_q_metrics *get_q_metrics)
{
    int ret_code = SUCCESS; /* set to success and see if any failure occurs */
    u16 q_id;               /* tmp variable for q id                        */
    u16 i = 0;              /* increments in the metrics list               */
    u8  q_id_found = 0;     /* flag to indicate if q id exists in list      */
    struct    nvme_gen_cq  *pcq_metrics;  /* params in nvme_gen_sq           */
    struct    nvme_gen_sq  *psq_metrics;  /* params in nvme_gen_sq           */
    struct    metrics_device_list *ptmp; /* pointer to metrics_device_list  */
    u8 __user *datap = (u8 __user *)get_q_metrics->buffer; /* lcl usr buff  */

    /* Get the q_id to lcl var */
    q_id = get_q_metrics->q_id;

    /* pointer to metrics_device_list structure. */
    ptmp = pmetrics_device_list;

    /* Determine the type of Q for which the metrics was needed */
    if (get_q_metrics->type == METRICS_SQ) {

        /* Determine the SQ Metrics */
        LOG_DBG("SQ Metrics requested.");

        /* Allocate kernel mem to pointer to public sq when needed */
        psq_metrics = kmalloc(sizeof(struct nvme_gen_sq), GFP_KERNEL);
        if (!psq_metrics) {
            LOG_ERR("failed mem allocation in sq metrics!!");
            return -ENOMEM;
        }

        /* Check if Q was admin Q? */
        if (get_q_metrics->q_id == 0) {
            LOG_DBG("Admin SQ Metrics...");
         } else {
            LOG_DBG("IO SQ Metrics...");
        }

        /* Search in the data structures for sq_id match */
        q_id_found = 0;
        i = 0;
        /* while device exists */
        while (ptmp != NULL) {
            /* while metrics_sq_list has an element */
            while (&ptmp->metrics_sq_list[i] != NULL) {
                /* match the q_id in the list */
                if (q_id == ptmp->metrics_sq_list[i].public_sq.sq_id) {
                    /* Set to found and point to the required data */
                    q_id_found = 1;
                    psq_metrics = &ptmp->metrics_sq_list[i].public_sq;

                    /* done with searching so exiting */
                    break;
                }
                /* point to next metrics sq element */
                i++;
            }
            /* check if above logic has detected our desired item */
            if (q_id_found == 1) {
                LOG_DBG("Found SQ_ID = %d at location = %d", q_id, i);
                break;
            }
            /* point to next device */
            ptmp++;
        }

        /* Check if the sq id requested exits? */
        if (!q_id_found) {
            LOG_ERR("The SQ_ID = %d is not in the list...", q_id);
            return -EINVAL;
        }

        LOG_DBG("If seg fault occurs here, then problem with user app...");
        LOG_DBG("Allocate user buffer with sufficient memory...");
        /* Copy to user space linked pointer buffer */
        memcpy((u8 *)&datap[0], (u8 *)psq_metrics, sizeof(struct nvme_gen_sq));

        /* Copy data to user space */
        ret_code = copy_to_user(&get_q_metrics->buffer[0], datap,
                     sizeof(struct nvme_gen_sq));

        /* Check if copy succeeded */
        if (ret_code < 0) {
            LOG_ERR("Error in copying SQ id %d metrics.. ",
                     get_q_metrics->q_id);
            return -EINVAL;
         }

        /* Done with q metrics requested, so return */
        return ret_code;
    } else if (get_q_metrics->type == METRICS_CQ) {

        /* Determine the CQ Metrics */
        LOG_DBG("CQ Metrics requested.");

        /* Allocate kernel mem to pointer to public cq when needed */
        pcq_metrics = kmalloc(sizeof(struct nvme_gen_cq), GFP_KERNEL);
        if (!pcq_metrics) {
            LOG_ERR("failed mem allocation in cq metrics!!");
            return -ENOMEM;
        }

        /* Check if Q was admin Q? */
        if (get_q_metrics->q_id == 0) {
            LOG_DBG("Admin CQ Metrics..");
        } else {
            LOG_DBG("IO CQ Metrics...");
        }

        /* Search in the data structures for cq_id match */
        q_id_found = 0;
        i = 0;
        while (ptmp != NULL) {
            while (&ptmp->metrics_cq_list[i] != NULL) {
                if (q_id == ptmp->metrics_cq_list[i].public_cq.q_id) {
                    q_id_found = 1;
                    pcq_metrics = &ptmp->metrics_cq_list[i].public_cq;
                    break;
                }
                i++;
            }
            if (q_id_found == 1) {
                LOG_DBG("Found CQ_ID = %d at location = %d", q_id, i);
                break;
            }
            ptmp++;
        }

        /* Check if the q id requested exits? */
        if (!q_id_found) {
            LOG_ERR("The CQ_ID = %d is not in the list...", q_id);
            return -EINVAL;
        }

        /* Copy to user space linked pointer buffer */
        memcpy((u8 *)&datap[0], (u8 *)pcq_metrics, sizeof(struct nvme_gen_cq));

        /* Copy data to user space */
        ret_code = copy_to_user(&get_q_metrics->buffer[0], datap,
                     sizeof(struct nvme_gen_cq));

        /* Check if copy succeeded */
        if (ret_code < 0) {
            LOG_ERR("Error in copying SQ id %d metrics.. ",
                     get_q_metrics->q_id);
            return -EINVAL;
        }
        /* Done with q metrics requested, so return */
        return ret_code;
    } else {
        /* The Q type is not SQ or CQ, so error out */
        LOG_ERR("Error in metrics Type...");
        LOG_ERR("Metrics Type: METRICS_SQ/METRICS_CQ only");
        return -EINVAL;
    }

    return SUCCESS;
}

