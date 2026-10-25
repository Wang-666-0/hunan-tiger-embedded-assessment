#include "dm_diag.h"
#include "app_config.h"
#include <string.h>

volatile uint32_t test_checks, test_failures, test_failed_line, test_done, test_stage;
#define CHECK(expr) do { ++test_checks; if (!(expr)) { ++test_failures; if (!test_failed_line) { test_failed_line = __LINE__; } } } while (0)

void test_run(void)
{
    CanFrame query, reply, untouched;
    uint32_t i, j, wanted;

    test_stage = 1U;
    CHECK(DmDiag_ReadableRidCount == 14U);
    CHECK(DmDiag_BuildRead(DM_DIAG_RID_PMAX, 0) == 0U);
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_PMAX, 0) == 0U);
    for (i = 0U; i < 256U; ++i) {
        wanted = 0U;
        for (j = 0U; j < DmDiag_ReadableRidCount; ++j) {
            if (DmDiag_ReadableRids[j] == i) { wanted = 1U; }
        }
        memset(&query, 0xA5, sizeof(query)); untouched = query;
        CHECK(DmDiag_IsReadableRid((uint8_t)i) == wanted);
        CHECK(DmDiag_BuildRead((uint8_t)i, &query) == wanted);
        if (!wanted) {
            CHECK(memcmp(&query, &untouched, sizeof(query)) == 0);
            continue;
        }
        CHECK(CanFrame_IsValid(&query));
        CHECK(query.bus == 1U && query.extended == 0U && query.id == 0x7FFU && query.dlc == 8U);
        CHECK(query.data[0] == 2U && query.data[1] == 0U && query.data[2] == 0x33U && query.data[3] == i);
        CHECK(query.data[4] == 0U && query.data[5] == 0U && query.data[6] == 0U && query.data[7] == 0U);
        CHECK(query.received_ms == 0U && query.fault_epoch == 0U);
        /* 读取请求前四字节与回复相同，不能认领读取请求本身。 */
        CHECK(DmDiag_MatchesReply((uint8_t)i, &query) == 0U);
        reply = query; reply.id = 0U;
        reply.data[4] = 0x42U; reply.data[5] = 0x51U; reply.data[6] = 0x62U; reply.data[7] = 0x73U;
        CHECK(DmDiag_MatchesReply((uint8_t)i, &reply) == 1U);
        reply.id = 0x302U;
        CHECK(DmDiag_MatchesReply((uint8_t)i, &reply) == 1U);
    }

    test_stage = 2U;
    CHECK(DmDiag_BuildRead(DM_DIAG_RID_MASTER_ID, &reply) == 1U); reply.id = 0x42U;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 1U);
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_SLAVE_ID, &reply) == 0U);
    CHECK(DmDiag_MatchesReply(0xFFU, &reply) == 0U);
    reply.data[0] = 1U; CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 0U);
    reply.data[0] = 2U; reply.data[1] = 1U;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 0U);
    reply.data[1] = 0U; reply.data[2] = 0x55U;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 0U);
    reply.data[2] = 0x33U; reply.data[3] = 0U;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 0U);
    reply.data[3] = DM_DIAG_RID_MASTER_ID; reply.bus = 2U;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 0U);
    reply.bus = 1U; reply.extended = 1U;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 0U);
    reply.extended = 0U; reply.dlc = 7U;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 0U);
    reply.dlc = 9U; CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 0U);
    reply.dlc = 8U; reply.id = 0x800U;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 0U);
    reply.id = 0x7FEU; CHECK(DmDiag_MatchesReply(DM_DIAG_RID_MASTER_ID, &reply) == 1U);

    test_stage = 3U;
    /* 常规 MIT 状态反馈中的从机/错误字节和位置量不是参数头。 */
    memset(&reply, 0, sizeof(reply)); reply.bus = 1U; reply.id = 0U; reply.dlc = 8U;
    reply.data[0] = 0x12U; reply.data[1] = 0x80U; reply.data[2] = 0x00U;
    reply.data[3] = 0x80U; reply.data[4] = 0x08U; reply.data[5] = 0x00U;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_PMAX, &reply) == 0U);
    reply.data[2] = 0x33U; reply.data[3] = DM_DIAG_RID_PMAX;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_PMAX, &reply) == 0U);
    reply.data[0] = 2U; reply.data[1] = 0U; reply.data[2] = 0xCCU;
    CHECK(DmDiag_MatchesReply(DM_DIAG_RID_PMAX, &reply) == 0U);
    test_done = 1U;
}
