#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "definitions.h"
#include "sysdnvme.h"
#include "dnvme_ds.h"

/*
 * nvme_get_q_metrics will return the q metrics from the global data
 * structures.
 */
int nvme_get_q_metrics(struct nvme_get_q_metrics *get_q_metrics)
{
    int ret_code = SUCCESS;

    if (get_q_metrics->type == METRICS_SQ) {
        LOG_DBG("SQ Metrics requested.");

        if (get_q_metrics->q_id == 0) {
            LOG_DBG("Admin SQ Metrics...");
        } else {
            LOG_DBG("IO SQ Metrics...");
        }
    } else if (get_q_metrics->type == METRICS_CQ) {
        LOG_DBG("CQ Metrics requested.");

        if (get_q_metrics->q_id == 0) {
            LOG_DBG("Admin CQ Metrics..");
        } else {
            LOG_DBG("IO CQ Metrics...");
        }
    } else {
        LOG_ERR("Error in metrics Type...");
        LOG_ERR("Metrics Type: METRICS_SQ/METRICS_CQ only");
        return -EINVAL;
    }

    return ret_code;
}
