/* -------------------------------------------------------------
 * PKTDMA (UDMA) TX: minimal, annotated golden path (TX-only)
 * Peripheral: FIFO-based TX (e.g., McASP TX)
 * Model:   One Host Packet Descriptor (HPD) + DMA_COMPLETION event
 * Rings:   We push HPD to the TX Free-Queue (FQ) ring; no TX CQ read
 * Event:   SHARED child event under a global master (SDK-like)
 * ------------------------------------------------------------- */

/* --------- 0) UDMA driver bring-up (once at boot) --------- */
/* Creates the driver + the global master event (needed for SHARED) */
Udma_InitPrms init;
UdmaInitPrms_init(UDMA_INST_ID_PKTDMA_0, &init);
init.skipGlobalEventReg = FALSE;                           /* create the master event */
DebugP_assert(Udma_init(&gUdmaDrvObj[0], &init) == UDMA_SOK);

Udma_DrvHandle drv  = &gUdmaDrvObj[0];                    /* driver "handle" */
Udma_ChObj     gTxChObj;  Udma_ChHandle     txCh  = &gTxChObj;   /* channel object + handle */
Udma_EventObj  gTxEvtObj; Udma_EventHandle  txEvt = &gTxEvtObj;  /* event object + handle */

/* (Optional) quick sanity */
DebugP_log("[UDMA] instType=%u skipGlobal=%u master=%p\n",
           drv->instType, drv->initPrms.skipGlobalEventReg,
           Udma_eventGetGlobalHandle(drv));

/* --------- 1) Open a PDMA TX channel (bind to peripheral TX request) --------- */
/* peerChNum is the SoC-defined PDMA thread for your peripheral TX */
Udma_ChPrms   ch;  Udma_ChTxPrms tx;
UdmaChPrms_init(&ch, UDMA_CH_TYPE_PDMA_TX);
UdmaChTxPrms_init(&tx);
ch.peerChNum = UDMA_PDMA_CH_MAIN0_MCASP1_TX;              /* <-- set to your peripheral */
DebugP_assert(Udma_chOpen(drv, txCh, UDMA_CH_TYPE_PDMA_TX) == UDMA_SOK);
DebugP_assert(Udma_chConfigTx(txCh, &ch, &tx)             == UDMA_SOK);

/* --------- 2) Configure PDMA pacing (how the FIFO is serviced) --------- */
/* elemSize: width per beat; elemCnt: water-level; fifoCnt: “groups” of elemCnt per submit */
Udma_ChPdmaPrms pd;
UdmaChPdmaPrms_init(&pd);
pd.elemSize = UDMA_PDMA_ES_32BITS;                        /* 32-bit FIFO access (typical) */
pd.elemCnt  = 16;                                         /* FIFO water-level (tune to HW) */
pd.fifoCnt  = 64/16;                                      /* not critical; pacing granularity */
pd.acc32    = 1;                                          /* accelerator on */
pd.burst    = 1;                                          /* burst on */
DebugP_assert(Udma_chConfigPdma(txCh, &pd) == UDMA_SOK);

/* --------- 3) Register a DMA_COMPLETION event (interrupt) --------- */
/* SHARED mode (SDK style): child event uses the master’s Hwi */
Udma_EventPrms ev;
UdmaEventPrms_init(&ev);
ev.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;    /* PKTDMA = completion events */
ev.eventMode         = UDMA_EVENT_MODE_SHARED;            /* child under global master */
ev.masterEventHandle = Udma_eventGetGlobalHandle(drv);    /* MUST be non-NULL in SHARED mode */
ev.chHandle          = txCh;                              /* event is for this channel */
ev.eventCb           = TxIsr;                             /* your ISR callback */
ev.appData           = ctx;                               /* your context (e.g., McASP handle) */
DebugP_assert(Udma_eventRegister(drv, txEvt, &ev) == UDMA_SOK);
DebugP_assert(Udma_eventEnable(txEvt)             == UDMA_SOK);

/* (Note) If you prefer EXCLUSIVE (dedicated IRQ):
   ev.eventMode = UDMA_EVENT_MODE_EXCLUSIVE;
   ev.masterEventHandle = NULL;  // must be NULL for EXCLUSIVE
*/

/* --------- 4) Build exactly ONE Host Packet Descriptor (HPD) --------- */
/* HPD “envelope” points to your audio buffer in RAM (physical address) */
extern uint8_t  txHpd[];    /* 64B-aligned buffer sized to UDMA_ALIGN_SIZE(sizeof(CSL_UdmapCppi5HMPD)) */
extern void    *buf;        /* your audio buffer (virtual address) */
extern uint32_t len;        /* bytes to send in this packet */

uint64_t bufPA = (uint64_t)Udma_defaultVirtToPhyFxn(buf,   0, NULL);
uint64_t hpdPA = (uint64_t)Udma_defaultVirtToPhyFxn(txHpd, 0, NULL);

CSL_UdmapCppi5HMPD *hpd = (CSL_UdmapCppi5HMPD *)txHpd;
const uint32_t dtype     = CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST;
memset(hpd, 0, sizeof(*hpd));
CSL_udmapCppi5SetDescType(hpd, dtype);
CSL_udmapCppi5SetPktLen(hpd, dtype, len);
CSL_udmapCppi5SetIds(hpd, dtype, 0x1, UDMA_DEFAULT_FLOW_ID);
CSL_udmapCppi5SetReturnPolicy(hpd, dtype, 0,0,0,0);
CSL_udmapCppi5SetBufferAddr(hpd, bufPA);
CSL_udmapCppi5SetBufferLen(hpd, len);
CSL_udmapCppi5SetOrgBufferAddr(hpd, bufPA);
CSL_udmapCppi5SetOrgBufferLen(hpd, len);

/* *** critical: write back HPD + payload before queueing *** */
CacheP_wb(hpd, sizeof(txHpd), CacheP_TYPE_ALLD);
CacheP_wb(buf, len,          CacheP_TYPE_ALLD);

/* --------- 5) Prime (queue the HPD), then enable channel, then doorbell(1) --------- */
/* Queue to the TX Free-Queue ring; doorbell count = number of descriptors you queued (here: 1) */
DebugP_assert(Udma_ringQueueRaw(Udma_chGetFqRingHandle(txCh), hpdPA) == UDMA_SOK);
DebugP_assert(Udma_chEnable(txCh) == UDMA_SOK);
Udma_ringSetDoorBell(Udma_chGetFqRingHandle(txCh), 1);
/* At this point, once the peripheral is out of reset and clocking, DMA will fetch HPD and start. */

/* --------- 6) ISR (DMA_COMPLETION) — rebuild HPD with next buffer and requeue --------- */
/* No TX CQ pop is needed for channel DMA_COMPLETION; just re-arm FQ with next packet. */
static void TxIsr(Udma_EventHandle e, uint32_t et, void *app)
{
    (void)e; (void)et;
    /* 1) (optional) notify app that the previous buffer finished */

    /* 2) choose next buffer (or loopjob), get PA + size */
    void    *nextBuf = get_next_tx_buffer(app);
    uint32_t nextLen = get_next_tx_len(app);
    uint64_t nextPA  = (uint64_t)Udma_defaultVirtToPhyFxn(nextBuf, 0, NULL);

    /* 3) rebuild the same HPD with the next buffer */
    CSL_UdmapCppi5HMPD *hpd = (CSL_UdmapCppi5HMPD *)txHpd;
    memset(hpd, 0, sizeof(*hpd));
    CSL_udmapCppi5SetDescType(hpd, dtype);
    CSL_udmapCppi5SetPktLen(hpd, dtype, nextLen);
    CSL_udmapCppi5SetIds(hpd,  dtype, 0x1, UDMA_DEFAULT_FLOW_ID);
    CSL_udmapCppi5SetReturnPolicy(hpd, dtype, 0,0,0,0);
    CSL_udmapCppi5SetBufferAddr(hpd, nextPA);
    CSL_udmapCppi5SetBufferLen(hpd, nextLen);
    CSL_udmapCppi5SetOrgBufferAddr(hpd, nextPA);
    CSL_udmapCppi5SetOrgBufferLen(hpd, nextLen);

    CacheP_wb(hpd,     sizeof(txHpd), CacheP_TYPE_ALLD);
    CacheP_wb(nextBuf, nextLen,       CacheP_TYPE_ALLD);

    /* 4) requeue same HPD to FQ and doorbell once */
    uint64_t hpdPA = (uint64_t)Udma_defaultVirtToPhyFxn(txHpd, 0, NULL);
    (void)Udma_ringQueueRaw(Udma_chGetFqRingHandle(txCh), hpdPA);
    Udma_ringSetDoorBell(Udma_chGetFqRingHandle(txCh), 1);
}