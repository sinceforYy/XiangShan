/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.mem

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import utility._
import xiangshan.ExceptionNO._
import xiangshan._
import xiangshan.backend.fu.PMPRespBundle
import xiangshan.backend.rob.{DebugLsInfoBundle, LsTopdownInfo, RobPtr}
import xiangshan.cache._
import xiangshan.cache.dcache.ReplayCarry
import xiangshan.cache.mmu.{TlbCmd, TlbReq, TlbRequestIO, TlbResp}
import xiangshan.mem.mdp._

class LoadToLsqReplayIO(implicit p: Parameters) extends XSBundle with HasDCacheParameters {
  // mshr refill index
  val missMSHRId = UInt(log2Up(cfg.nMissEntries).W)
  // get full data from store queue and sbuffer
  val canForwardFullData = Bool()
  // wait for data from store inst's store queue index
  val dataInvalidSqIdx = new SqPtr
  // wait for address from store queue index
  val addrInvalidSqIdx = new SqPtr
  // replay carry
  val replayCarry = new ReplayCarry
  // data in last beat
  val dataInLastBeat = Bool()
  // replay cause
  val cause = Vec(LoadReplayCauses.allCauses, Bool())
  //
  // performance debug information
  val debug = new PerfDebugInfo

  // 
  def tlbMiss       = cause(LoadReplayCauses.tlbMiss)
  def waitStore     = cause(LoadReplayCauses.waitStore)
  def schedError    = cause(LoadReplayCauses.schedError)
  def rarReject     = cause(LoadReplayCauses.rarReject)
  def rawReject     = cause(LoadReplayCauses.rawReject)
  def dcacheMiss    = cause(LoadReplayCauses.dcacheMiss)
  def bankConflict  = cause(LoadReplayCauses.bankConflict)
  def dcacheReplay  = cause(LoadReplayCauses.dcacheReplay)
  def forwardFail   = cause(LoadReplayCauses.forwardFail)

  def forceReplay() = rarReject || rawReject || schedError || waitStore || tlbMiss
  def needReplay()  = cause.asUInt.orR 
}

class LoadToReplayIO(implicit p: Parameters) extends XSBundle {
  val req = ValidIO(new LqWriteBundle)
  val resp = Input(UInt(log2Up(LoadQueueReplaySize).W))
}

class LoadToLsqIO(implicit p: Parameters) extends XSBundle {
  val loadIn = DecoupledIO(new LqWriteBundle)
  val loadOut = Flipped(DecoupledIO(new ExuOutput))
  val ldRawData = Input(new LoadDataFromLQBundle)
  val forward = new PipeLoadForwardQueryIO
  val stldVioQuery = new LoadViolationQueryIO
  val ldldVioQuery = new LoadViolationQueryIO
  val trigger = Flipped(new LqTriggerIO)
}

class LoadToLoadIO(implicit p: Parameters) extends XSBundle {
  // load to load fast path is limited to ld (64 bit) used as vaddr src1 only
  val data = UInt(XLEN.W)
  val valid = Bool()
}

class LoadUnitTriggerIO(implicit p: Parameters) extends XSBundle {
  val tdata2 = Input(UInt(64.W))
  val matchType = Input(UInt(2.W))
  val tEnable = Input(Bool()) // timing is calculated before this
  val addrHit = Output(Bool())
  val lastDataHit = Output(Bool())
}

class LoadUnit(implicit p: Parameters) extends XSModule
  with HasLoadHelper
  with HasPerfEvents
  with HasDCacheParameters
  with HasCircularQueuePtrHelper
{
  val io = IO(new Bundle() {
    val loadIn = Flipped(Decoupled(new ExuInput))
    val loadOut = Decoupled(new ExuOutput)
    val rsIdx = Input(UInt())
    val redirect = Flipped(ValidIO(new Redirect))
    val isFirstIssue = Input(Bool())
    val dcache = new DCacheLoadIO
    val sbuffer = new LoadForwardQueryIO
    val lsq = new LoadToLsqIO
    val tlDchannel = Input(new DcacheToLduForwardIO)
    val forward_mshr = Flipped(new LduToMissqueueForwardIO)
    val refill = Flipped(ValidIO(new Refill))
    val fastUop = ValidIO(new MicroOp) // early wakeup signal generated in load_s1, send to RS in load_s2
    val trigger = Vec(3, new LoadUnitTriggerIO)

    val tlb = new TlbRequestIO(2)
    val pmp = Flipped(new PMPRespBundle()) // arrive same to tlb now

    // provide prefetch info
    val prefetch_train = ValidIO(new LdPrefetchTrainBundle())

    // hardware prefetch to l1 cache req
    val prefetch_req = Flipped(ValidIO(new L1PrefetchReq))

    // load to load fast path
    val fastpathOut = Output(new LoadToLoadIO)
    val fastpathIn = Input(new LoadToLoadIO)
    val loadFastMatch = Input(Bool())
    val loadFastImm = Input(UInt(12.W))

    // rs feedback
    val feedbackFast = ValidIO(new RSFeedback) // stage 2
    val feedbackSlow = ValidIO(new RSFeedback) // stage 3

    // load ecc
    val s3_delayedLoadError = Output(Bool()) // load ecc error
    // Note that io.s3_delayed_load_error and io.lsq.s3_delayed_load_error is different

    // load unit ctrl
    val csrCtrl = Flipped(new CustomCSRCtrlIO)

    val reExecuteQuery = Flipped(Vec(StorePipelineWidth, Valid(new LoadReExecuteQueryIO)))    // load replay
    val replay = Flipped(Decoupled(new LsPipelineBundle))
    val debug_ls = Output(new DebugLsInfoBundle)
    val lsTopdownInfo = Output(new LsTopdownInfo)
    val s2IsPointerChasing = Output(Bool()) // provide right pc for hw prefetch
    val lqReplayFull = Input(Bool())

    // Load fast replay path
    val fastReplayIn = Flipped(Decoupled(new LqWriteBundle))
    val fastReplayOut = Decoupled(new LqWriteBundle)

    val l2Hint = Input(Valid(new L2ToL1Hint))    
  })

  require(LoadPipelineWidth == exuParameters.LduCnt)

  val s1_ready, s2_ready, s3_ready = WireInit(false.B)

  // Pipeline
  // --------------------------------------------------------------------------------
  // stage 0
  // --------------------------------------------------------------------------------
  // generate addr, use addr to query DCache and DTLB
  val s0_valid              = Wire(Bool())
  val s0_vaddr              = WireInit(UInt(VAddrBits.W))
  val s0_mask               = Wire(UInt(8.W))
  val s0_uop                = Wire(new MicroOp)
  val s0_hasROBEntry        = Wire(Bool())
  val s0_rsIdx              = Wire(UInt(log2Up(IssQueSize).W))
  val s0_sqIdx              = Wire(new SqPtr)
  val s0_mshrid             = Wire(UInt())
  val s0_tryFastPath        = Wire(Bool())
  val s0_replayCarry        = Wire(new ReplayCarry)
  val s0_isFirstIssue       = Wire(Bool())
  val s0_isLoadReplay       = Wire(Bool())
  val s0_sleepIndex         = Wire(UInt())
  val s0_replacementUpdated = Wire(Bool())
  val s0_can_go             = s1_ready
  val s0_fire               = s0_valid && s0_can_go
  val s0_out                = Wire(new LqWriteBundle)

  // load flow select/gen
  // src0: load replayed by LSQ (io.replay)
  // src1: hardware prefetch from prefetchor (high confidence) (io.prefetch)
  // src2: int read / software prefetch first issue from RS (io.in)
  // src3: vec read first issue from RS (TODO)
  // src4: load try pointchaising when no issued or replayed load (io.fastpath)
  // src5: hardware prefetch from prefetchor (high confidence) (io.prefetch)
  // priority: high to low
  val s0_replayShouldStall = io.loadIn.valid && isAfter(io.replay.bits.uop.robIdx, io.loadIn.bits.uop.robIdx)
  val lfsrc_requests = VecInit(Seq(
    // fast replay path
    io.fastReplayIn.valid, 
    // queue-based replay path
    io.replay.valid && !s0_replayShouldStall,
    // high confidence prefetch request
    io.prefetch_req.valid && io.prefetch_req.bits.confidence > 0.U,
    // issue int load path
    io.loadIn.valid,
    // issue vector load path 
    false.B,
    // load-to-load forward fast path
    io.fastpathIn.valid,
    // load confidence prefetch request
    io.prefetch_req.valid && io.prefetch_req.bits.confidence === 0.U
  ))
  val lfsrc_readys = (0 until lfsrc_requests.length).map(i => if (i == 0) true.B else lfsrc_requests.take(i).reduce(!_ && !_))
  val lfsrc_selects = lfsrc_requests.zip(lfsrc_readys).map { case (vld, rdy) => vld && rdy }
  val lfsrc_loadFastReplay_ready :: lfsrc_loadReplay_ready :: lfsrc_highconfhwPrefetch_ready :: lfsrc_intloadFirstIssue_ready :: lfsrc_vecloadFirstIssue_ready :: lfsrc_l2lForward_ready :: lfsrc_lowconfhwPrefetch_ready :: Nil = lfsrc_readys
  val lfsrc_loadFastReplay_select :: lfsrc_loadReplay_select :: lfsrc_highconfhwPrefetch_select :: lfsrc_intloadFirstIssue_select :: lfsrc_vecloadFirstIssue_select :: lfsrc_l2lForward_select :: lfsrc_lowconfhwPrefetch_select :: Nil = lfsrc_selects
  val lfsrc_hwprefetch_select = lfsrc_highconfhwPrefetch_select || lfsrc_lowconfhwPrefetch_select
  assert(!lfsrc_vecloadFirstIssue_select) // to be added

  val s0_l2lForward_select = lfsrc_l2lForward_select
  s0_valid := lfsrc_selects.reduce(_|_) && io.dcache.req.ready && !s0_kill

  // which is S0's out is ready and dcache is ready
  val s0_tryPointerChasing   = lfsrc_l2lForward_select
  val s0_doTryPointerChasing = s0_tryPointerChasing && s0_can_go && io.dcache.req.ready
  val s0_pointerChasingVAddr = io.fastpathIn.data(5, 0) +& io.loadFastImm(5, 0)
  val s0_kill                = s0_out.uop.robIdx.needFlush(io.redirect)

  // prefetch related ctrl signal
  val s0_isPrefetch      = Wire(Bool())
  val s0_isPrefetchRead  = Wire(Bool())
  val s0_isPrefetchWrite = Wire(Bool())
  val s0_isHWPrefetch    = lfsrc_hwprefetch_select

  // query DTLB 
  io.tlb.req.valid                   := s0_valid 
  io.tlb.req.bits.cmd                := Mux(s0_isPrefetch,
                                         Mux(s0_isPrefetchWrite, TlbCmd.write, TlbCmd.read),
                                         TlbCmd.read
                                       )
  io.tlb.req.bits.vaddr              := Mux(lfsrc_hwprefetch_select, io.prefetch_req.bits.paddr, s0_vaddr)
  io.tlb.req.bits.size               := LSUOpType.size(s0_uop.ctrl.fuOpType)
  io.tlb.req.bits.kill               := DontCare
  io.tlb.req.bits.memidx.is_ld       := true.B
  io.tlb.req.bits.memidx.is_st       := false.B
  io.tlb.req.bits.memidx.idx         := s0_uop.lqIdx.value
  io.tlb.req.bits.debug.robIdx       := s0_uop.robIdx
  io.tlb.req.bits.no_translate       := lfsrc_hwprefetch_select  // hw b.reqetch addr does not need to be translated
  io.tlb.req.bits.debug.pc           := s0_uop.cf.pc
  io.tlb.req.bits.debug.isFirstIssue := s0_isFirstIssue

  // query DCache
  io.dcache.req.valid             := s0_valid
  io.dcache.req.bits.cmd          := Mux(s0_isPrefetchRead, 
                                      MemoryOpConstants.M_PFR, 
                                      Mux(s0_isPrefetchWrite, MemoryOpConstants.M_PFW, MemoryOpConstants.M_XRD)
                                    )
  io.dcache.req.bits.vaddr        := s0_vaddr
  io.dcache.req.bits.mask         := s0_mask
  io.dcache.req.bits.data         := DontCare
  io.dcache.req.bits.isFirstIssue := s0_isFirstIssue
  io.dcache.req.bits.instrtype    := Mux(s0_isPrefetch, DCACHE_PREFETCH_SOURCE.U, LOAD_SOURCE.U)
  io.dcache.req.bits.debug_robIdx := s0_uop.robIdx.value
  io.dcache.req.bits.replayCarry  := s0_replayCarry
  io.dcache.req.bits.id           := DontCare // TODO: update cache meta

  // load flow priority mux
  def fromNullSource() = {
    s0_vaddr           := 0.U
    s0_mask            := 0.U
    s0_uop             := 0.U.asTypeOf(new MicroOp)
    s0_hasROBEntry     := false.B
    s0_sqIdx           := 0.U.asTypeOf(new SqPtr)
    s0_rsIdx           := 0.U
    s0_replayCarry     := 0.U.asTypeOf(s0_replayCarry.cloneType)
    s0_mshrid          := 0.U
    s0_isFirstIssue    := false.B
    s0_isLoadReplay    := false.B
    s0_isPrefetch      := false.B
    s0_isPrefetchRead  := false.B
    s0_isPrefetchWrite := false.B
    s0_sleepIndex      := 0.U
  }

  def fromFastReplaySource(src: LqWriteBundle) = {
    s0_vaddr           := src.vaddr
    s0_mask            := src.mask 
    s0_uop             := src.uop
    s0_hasROBEntry     := src.hasROBEntry
    s0_sqIdx           := src.uop.sqIdx 
    s0_replayCarry     := src.replayInfo.replayCarry
    s0_mshrid          := src.replayInfo.missMSHRId
    s0_rsIdx           := src.rsIdx 
    s0_isFirstIssue    := false.B 
    s0_isLoadReplay    := src.isLoadReplay
    s0_isPrefetch      := LSUOpType.isPrefetch(src.uop.ctrl.fuOpType) 
    s0_isPrefetchRead  := src.uop.ctrl.fuOpType === LSUOpType.prefetch_r
    s0_isPrefetchWrite := src.uop.ctrl.fuOpType === LSUOpType.prefetch_w
    s0_sleepIndex      := src.sleepIndex
  }

  def fromQueueBasedReplaySource(src: LsPipelineBundle) = {
    s0_vaddr           := src.vaddr
    s0_mask            := genWmask(src.vaddr, src.uop.ctrl.fuOpType(1, 0))
    s0_uop             := src.uop
    s0_hasROBEntry     := true.B
    s0_sqIdx           := src.uop.sqIdx
    s0_rsIdx           := src.rsIdx
    s0_replayCarry     := src.replayCarry
    s0_mshrid          := src.mshrid
    s0_isFirstIssue    := src.isFirstIssue
    s0_isLoadReplay    := true.B
    s0_isPrefetch      := LSUOpType.isPrefetch(src.uop.ctrl.fuOpType)
    s0_isPrefetchRead  := src.uop.ctrl.fuOpType === LSUOpType.prefetch_r
    s0_isPrefetchWrite := src.uop.ctrl.fuOpType === LSUOpType.prefetch_w
    s0_sleepIndex      := src.sleepIndex
  }

  def fromPrefetchSource(src: L1PrefetchReq) = {
    s0_vaddr           := src.getVaddr()
    s0_mask            := 0.U
    s0_uop             := DontCare
    s0_hasROBEntry     := false.B
    s0_sqIdx           := DontCare
    s0_rsIdx           := DontCare
    s0_replayCarry     := DontCare
    s0_mshrid          := DontCare
    s0_isFirstIssue    := false.B
    s0_isLoadReplay    := false.B
    s0_isPrefetch      := true.B
    s0_isPrefetchRead  := !src.is_store
    s0_isPrefetchWrite := src.is_store    
    s0_sleepIndex      := DontCare
  }

  def fromIntIssueSource(src: ExuInput) = {
    s0_vaddr           := src.src(0) + SignExt(src.uop.ctrl.imm(11, 0), VAddrBits)
    s0_mask            := genWmask(s0_vaddr, src.uop.ctrl.fuOpType(1,0))
    s0_uop             := src.uop
    s0_hasROBEntry     := true.B
    s0_sqIdx           := src.uop.sqIdx
    s0_rsIdx           := io.rsIdx
    s0_replayCarry     := DontCare
    s0_mshrid          := DontCare
    s0_isFirstIssue    := true.B
    s0_isLoadReplay    := false.B
    s0_isPrefetch      := LSUOpType.isPrefetch(src.uop.ctrl.fuOpType)
    s0_isPrefetchRead  := src.uop.ctrl.fuOpType === LSUOpType.prefetch_r
    s0_isPrefetchWrite := src.uop.ctrl.fuOpType === LSUOpType.prefetch_w
    s0_sleepIndex      := DontCare
  }

  def fromVecIssueSource() = {
    //  TODO
    assert(false, "TODO!")
  }

  def fromLoadToLoadSource(src: LoadToLoadIO) = {
    s0_vaddr              := Cat(io.fastpathIn.data(XLEN-1, 6), s0_pointerChasingVAddr(5,0))
    s0_tryFastPath        := lfsrc_l2lForward_select
    // When there's no valid instruction from RS and LSQ, we try the load-to-load forwarding.
    // Assume the pointer chasing is always ld.
    s0_uop.ctrl.fuOpType  := LSUOpType.ld
    s0_mask               := genWmask(0.U, LSUOpType.ld)
    // we dont care s0_isFirstIssue and s0_rsIdx and s0_sqIdx in S0 when trying pointchasing
    // because these signals will be updated in S1
    s0_sqIdx              := DontCare
    s0_rsIdx              := DontCare
    s0_mshrid             := DontCare
    s0_isFirstIssue       := true.B
    s0_isLoadReplay       := false.B
    s0_isPrefetch         := false.B
    s0_isPrefetchRead     := false.B 
    s0_isPrefetchWrite    := false.B
    s0_sleepIndex         := DontCare
  }

  when (lfsrc_loadFastReplay_select)         { fromFastReplaySource(io.fastReplayIn.bits)   } 
  .elsewhen (lfsrc_loadReplay_select)        { fromQueueBasedReplaySource(io.replay.bits)   } 
  .elsewhen (lfsrc_hwprefetch_select)        { fromPrefetchSource(io.prefetch_req.bits)     } 
  .elsewhen (lfsrc_intloadFirstIssue_select) { fromIntIssueSource(io.loadIn.bits)           } 
  .elsewhen (lfsrc_vecloadFirstIssue_select) { fromVecIssueSource()                         } 
  .otherwise {
    if (EnableLoadToLoadForward) {
      fromLoadToLoadSource(io.fastpathIn)
    } else {
      fromNullSource()
    }
  }

  // address align check
  val s0_addrAligned = LookupTree(s0_uop.ctrl.fuOpType(1, 0), List(
    "b00".U   -> true.B,                   //b
    "b01".U   -> (s0_vaddr(0)    === 0.U), //h
    "b10".U   -> (s0_vaddr(1, 0) === 0.U), //w
    "b11".U   -> (s0_vaddr(2, 0) === 0.U)  //d
  )) 

  // accept load flow if dcache ready (tlb is always ready)
  // TODO: prefetch need writeback to loadQueueFlag
  s0_out                    := DontCare
  s0_out.rsIdx              := s0_rsIdx
  s0_out.vaddr              := s0_vaddr
  s0_out.mask               := s0_mask
  s0_out.uop                := s0_uop
  s0_out.uop.cf.exceptionVec(loadAddrMisaligned) := !s0_addrAligned
  s0_out.isFirstIssue       := s0_isFirstIssue
  s0_out.hasROBEntry        := s0_hasROBEntry
  s0_out.isPrefetch         := s0_isPrefetch
  s0_out.isHWPrefetch       := s0_isHWPrefetch
  s0_out.isLoadReplay       := s0_isLoadReplay
  s0_out.mshrid             := s0_mshrid
  s0_out.forward_tlDchannel := io.replay.valid && io.replay.bits.forward_tlDchannel
  when(io.tlb.req.valid && s0_isFirstIssue) {
    s0_out.uop.debugInfo.tlbFirstReqTime := GTimer()
  }.otherwise{
    s0_out.uop.debugInfo.tlbFirstReqTime := s0_uop.debugInfo.tlbFirstReqTime
  }
  s0_out.sleepIndex         := s0_sleepIndex

  // load fast replay
  io.fastReplayIn.ready := (s1_ready && io.dcache.req.ready && lfsrc_loadFastReplay_ready)

  // load flow source ready
  // always accept load flow from load replay queue
  // io.replay has highest priority
  io.replay.ready := (s1_ready && io.dcache.req.ready && lfsrc_loadReplay_ready && !s0_replayShouldStall)

  // accept load flow from rs when:
  // 1) there is no lsq-replayed load
  // 2) there is no high confidence prefetch request
  io.loadIn.ready := (s1_ready && io.dcache.req.ready && lfsrc_intloadFirstIssue_ready)

  // for hw prefetch load flow feedback, to be added later
  // io.prefetch_in.ready := lfsrc_hwprefetch_select

  // dcache replacement extra info
  // TODO: should prefetch load update replacement?
  s0_replacementUpdated := Mux(lfsrc_loadReplay_select, io.replay.bits.replacementUpdated, false.B)


  XSDebug(io.dcache.req.fire,
    p"[DCACHE LOAD REQ] pc ${Hexadecimal(s0_uop.cf.pc)}, vaddr ${Hexadecimal(s0_vaddr)}\n"
  )
  XSDebug(s0_valid,
    p"S0: pc ${Hexadecimal(s0_out.uop.cf.pc)}, lId ${Hexadecimal(s0_out.uop.lqIdx.asUInt)}, " +
    p"vaddr ${Hexadecimal(s0_out.vaddr)}, mask ${Hexadecimal(s0_out.mask)}\n")

  // Pipeline
  // --------------------------------------------------------------------------------
  // stage 1
  // --------------------------------------------------------------------------------
  // TLB resp (send paddr to dcache) 
  val s1_valid  = RegInit(false.B)
  val s1_in     = Wire(new LqWriteBundle) 
  val s1_out    = Wire(new LqWriteBundle)
  val s1_kill   = Wire(Bool())
  val s1_can_go = s2_ready 
  val s1_fire   = s1_valid && !s1_kill && s1_can_go

  s1_ready := !s1_valid || s1_kill || s2_ready 
  when (s0_fire) { s1_valid := true.B } 
  .elsewhen (s1_fire) { s1_valid := false.B }
  .elsewhen (s1_kill) { s1_valid := false.B }
  s1_in   := RegEnable(s0_out, s0_fire)
  s1_kill := s1_in.uop.robIdx.needFlush(io.redirect)

  val s1_paddr_dup_lsu    = io.tlb.resp.bits.paddr(0)
  val s1_paddr_dup_dcache = io.tlb.resp.bits.paddr(1)
  val s1_exception        = ExceptionNO.selectByFu(s1_in.uop.cf.exceptionVec, lduCfg).asUInt.orR   // af & pf exception were modified below.
  val s1_tlb_miss         = io.tlb.resp.bits.miss 
  val s1_isPrefetch       = s1_in.isPrefetch
  val s1_isHWPrefetch     = s1_in.isHWPrefetch
  val s1_isSWPrefetch     = s1_isPrefetch && !s1_isHWPrefetch
  val s1_tlb_memidx       = io.tlb.resp.bits.memidx

  when (s1_tlb_memidx.is_ld && io.tlb.resp.valid && !s1_tlb_miss && s1_tlb_memidx.idx === s1_in.uop.lqIdx.value) {
    // printf("load idx = %d\n", s1_tlb_memidx.idx)
    s1_out.uop.debugInfo.tlbRespTime := GTimer()    
  }

  io.tlb.resp.ready := true.B 
  io.dcache.s1_paddr_dup_lsu <> s1_paddr_dup_lsu
  io.dcache.s1_paddr_dup_dcache <> s1_paddr_dup_dcache
  io.dcache.s1_kill <> s1_kill

  // store to load forwarding
  io.sbuffer.valid := s1_valid && !(s1_exception || s1_tlb_miss || s1_kill || s1_isPrefetch)
  io.sbuffer.vaddr := s1_in.vaddr
  io.sbuffer.paddr := s1_paddr_dup_lsu
  io.sbuffer.uop   := s1_in.uop
  io.sbuffer.sqIdx := s1_in.uop.sqIdx
  io.sbuffer.mask  := s1_in.mask 
  io.sbuffer.pc    := s1_in.uop.cf.pc // FIXME: remove it

  io.lsq.forward.valid     := s1_valid && !(s1_exception || s1_tlb_miss || s1_kill || s1_isPrefetch)
  io.lsq.forward.vaddr     := s1_in.vaddr 
  io.lsq.forward.paddr     := s1_paddr_dup_lsu
  io.lsq.forward.uop       := s1_in.uop
  io.lsq.forward.sqIdx     := s1_in.uop.sqIdx 
  io.lsq.forward.sqIdxMask := DontCare
  io.lsq.forward.mask      := s1_in.mask
  io.lsq.forward.pc        := s1_in.uop.cf.pc // FIXME: remove it

  // st-ld violation query
  val s1_sched_err = VecInit((0 until StorePipelineWidth).map(w => io.reExecuteQuery(w).valid &&
                          isAfter(s1_in.uop.robIdx, io.reExecuteQuery(w).bits.robIdx) && 
                          (s1_paddr_dup_lsu(PAddrBits-1, 3) === io.reExecuteQuery(w).bits.paddr(PAddrBits-1, 3)) &&
                          (s1_in.mask & io.reExecuteQuery(w).bits.mask).orR)).asUInt.orR && !s1_tlb_miss
  // Generate forwardMaskFast to wake up insts earlier 
  val s1_forwardMaskFast = ((~(io.lsq.forward.forwardMaskFast.asUInt | io.sbuffer.forwardMaskFast.asUInt)).asUInt & s1_in.mask) === 0.U

  s1_out         := s1_in
  s1_out.paddr   := s1_paddr_dup_lsu
  s1_out.tlbMiss := s1_tlb_miss
  s1_out.ptwBack := io.tlb.resp.bits.ptwBack
  s1_out.rsIdx   := s1_in.rsIdx
  // current ori test will cause the case of ldest == 0, below will be modifeid in the future.
  // af & pf exception were modified
  s1_out.uop.cf.exceptionVec(loadPageFault)   := io.tlb.resp.bits.excp(0).pf.ld
  s1_out.uop.cf.exceptionVec(loadAccessFault) := io.tlb.resp.bits.excp(0).af.ld
  s1_out.replayInfo.debug := s1_in.uop.debugInfo
  s1_out.replayInfo.cause(LoadReplayCauses.schedError) := s1_sched_err && !s1_isSWPrefetch

  // pointer chasing
  val s1_tryPointerChasing       = RegNext(s0_doTryPointerChasing, false.B)
  val s1_pointerChasingVAddr     = RegEnable(s0_pointerChasingVAddr, s0_doTryPointerChasing)
  val s1_fuOpTypeIsNotLd         = WireInit(false.B)
  val s1_notFastMatch            = WireInit(false.B)
  val s1_addressMisMatch         = WireInit(false.B)
  val s1_addressNotAligned       = WireInit(false.B)
  val s1_pointerChasingCancelled = WireInit(false.B)
  val s1_cancelPointerChasing    = WireInit(false.B)

  if (EnableLoadToLoadForward) {
    // Sometimes, we need to cancel the load-load forwarding.
    // These can be put at S0 if timing is bad at S1.
    // Case 0: CACHE_SET(base + offset) != CACHE_SET(base) (lowest 6-bit addition has an overflow)
    s1_addressMisMatch := s1_pointerChasingVAddr(6) || RegEnable(io.loadFastImm(11, 6).orR, s0_doTryPointerChasing)
    // Case 1: the address is not 64-bit aligned or the fuOpType is not LD
    s1_addressNotAligned := s1_pointerChasingVAddr(2, 0).orR
    s1_fuOpTypeIsNotLd  := io.loadIn.bits.uop.ctrl.fuOpType =/= LSUOpType.ld
    // Case 2: this is not a valid load-load pair
    s1_notFastMatch := RegEnable(!io.loadFastMatch, s0_tryPointerChasing)
    // Case 3: this load-load uop is cancelled
    s1_pointerChasingCancelled := !io.loadIn.valid

    when (s1_tryPointerChasing) {
      s1_cancelPointerChasing := s1_addressMisMatch || s1_addressNotAligned || s1_fuOpTypeIsNotLd || s1_notFastMatch || s1_pointerChasingCancelled

      s1_in.uop           := io.loadIn.bits.uop
      s1_in.rsIdx         := io.rsIdx
      s1_in.vaddr         := Cat(s1_in.vaddr(VAddrBits - 1, 6), s1_pointerChasingVAddr(5, 3), 0.U(3.W))
      s1_in.isFirstIssue  := io.isFirstIssue
      s1_paddr_dup_lsu    := Cat(io.tlb.resp.bits.paddr(0)(PAddrBits - 1, 6), s1_pointerChasingVAddr(5, 3), 0.U(3.W))
      s1_paddr_dup_dcache := Cat(io.tlb.resp.bits.paddr(0)(PAddrBits - 1, 6), s1_pointerChasingVAddr(5, 3), 0.U(3.W))

      // recored tlb time when get the data to ensure the correctness of the latency calculation (although it should not record in here, because it does not use tlb)
      s1_in.uop.debugInfo.tlbFirstReqTime := GTimer()
      s1_in.uop.debugInfo.tlbRespTime     := GTimer()
    }
    when (s1_cancelPointerChasing) {
      s1_kill := true.B
    }.otherwise {
      s0_kill := s1_tryPointerChasing && !io.replay.fire && !io.fastReplay.fire
      when (s1_tryPointerChasing) {
        io.loadIn.ready := true.B
      }
    }
  }

  XSDebug(s1_valid,
    p"S1: pc ${Hexadecimal(s1_out.uop.cf.pc)}, lId ${Hexadecimal(s1_out.uop.lqIdx.asUInt)}, tlb_miss ${io.tlb.resp.bits.miss}, " +
    p"paddr ${Hexadecimal(s1_out.paddr)}, mmio ${s1_out.mmio}\n")

  // Pipeline
  // --------------------------------------------------------------------------------
  // stage 2
  // --------------------------------------------------------------------------------
  // s2: DCache resp
  val s2_valid  = RegInit(false.B)
  val s2_can_go = s3_ready 
  val s2_fire   = s2_valid && s2_can_go
  val s2_in     = Wire(new LqWriteBundle) 
  val s2_out    = Wire(new LqWriteBundle)

  val s2_kill = s2_in.uop.robIdx.needFlush(io.redirect)
  s2_ready := !s1_valid || s2_kill || s2_ready 
  when (s1_fire) { s2_valid := true.B } 
  .elsewhen (s2_fire) { s2_valid := false.B }
  s2_in := RegEnable(s1_out, s1_fire)

  val s2_pmp = WireInit(io.pmp)
  val s2_static_pm = RegNext(io.tlb.resp.bits.static_pm)
  when (s2_static_pm.valid) {
    s2_pmp.ld    := false.B 
    s2_pmp.st    := false.B 
    s2_pmp.instr := false.B 
    s2_pmp.mmio  := s2_static_pm.bits
  }
  val s2_isPrefetch = s2_in.isPrefetch
  val s2_isHWPrefetch = s2_in.isHWPrefetch

  // exception that may cause load addr to be invalid / illegal
  // if such exception happen, that inst and its exception info
  // will be force writebacked to rob
  val s2_exception_vec = WireInit(s2_in.uop.cf.exceptionVec)
  s2_exception_vec(loadAccessFault) := s2_in.uop.cf.exceptionVec(loadAccessFault) || s2_pmp.ld

  // soft prefetch will not trigger any exception (but ecc error interrupt may be triggered)
  when (s2_isPrefetch || s2_in.tlbMiss) {
    s2_exception_vec := 0.U.asTypeOf(s2_exception_vec.cloneType)  
  }
  val s2_exception = ExceptionNO.selectByFu(s2_exception_vec, lduCfg).asUInt.orR 

  val (s2_forward_D, s2_forwardData_D) = io.tlDchannel.forward(s1_valid && s1_out.forward_tlDchannel, s1_out.mshrid, s1_out.paddr)
  val (s2_forward_result_valid, s2_forward_mshr, s2_forwardData_mshr) = io.forward_mshr.forward()
  val s2_forward_D_or_mshr_valid = s2_forward_result_valid && (s2_forward_D || s2_forward_mshr)
  val s2_cache_hit = io.dcache.s2_hit || s2_forward_D_or_mshr_valid

  // writeback access fault caused by ecc error / bus error
  // * ecc data error is slow to generate, so we will not use it until load stage 3
  // * in load stage 3, an extra signal io.load_error will be used to
  val s2_actually_mmio   = s2_pmp.mmio
  val s2_mmio            = !s2_isPrefetch && s2_actually_mmio && !s2_exception && !s2_in.tlbMiss
  val s2_cache_miss      = io.dcache.resp.bits.miss && !s2_forward_D_or_mshr_valid
  val s2_cache_replay    = io.dcache.resp.bits.replay && !s2_forward_D_or_mshr_valid
  val s2_cache_handled   = io.dcache.resp.bits.handled
  val s2_cache_tag_error = RegNext(io.csrCtrl.cache_error_enable) && io.dcache.resp.bits.tag_error
  val s2_forward_fail    = io.lsq.forward.matchInvalid || io.sbuffer.matchInvalid
  val s2_wait_store      = s2_in.uop.cf.storeSetHit && io.lsq.forward.addrInvalid && !s2_mmio && !s2_isPrefetch 
  val s2_data_invalid    = io.lsq.forward.dataInvalid && !s2_exception
  val s2_fullForward     = Wire(Bool())
  val s2_dcache_kill     = s2_pmp.ld || s2_pmp.mmio
  val s2_canReplayFromFetch = !s2_mmio && !s2_isPrefetch && !s2_in.tlbMiss

  io.dcache.resp.ready := true.B
  val s2_dcacheShouldResp = !(s2_in.tlbMiss || s2_exception || s2_mmio || s2_isPrefetch)
  assert(!(s2_valid && (s2_dcacheShouldResp && !io.dcache.resp.valid)), "DCache response got lost")

  // st-ld violation query
  //  NeedFastRecovery Valid when
  //  1. Fast recovery query request Valid.
  //  2. Load instruction is younger than requestors(store instructions).
  //  3. Physical address match.
  //  4. Data contains.
  val s2_sched_err = VecInit((0 until StorePipelineWidth).map(w => io.reExecuteQuery(w).valid &&
                              isAfter(s2_in.uop.robIdx, io.reExecuteQuery(w).bits.robIdx) &&
                              (s2_in.paddr(PAddrBits-1,3) === io.reExecuteQuery(w).bits.paddr(PAddrBits-1, 3)) &&
                              (s2_in.mask & io.reExecuteQuery(w).bits.mask).orR)).asUInt.orR && !s2_in.tlbMiss

  val s2_fast_replay = ((s2_sched_err || s2_in.replayInfo.cause(LoadReplayCauses.schedError)) ||
                       (!s2_wait_store &&
                       !s2_in.tlbMiss &&
                       s2_cache_replay) || 
                      (s2_out.miss && io.l2Hint.valid && (s2_out.replayInfo.missMSHRId === io.l2Hint.bits.sourceId))) &&
                       !s2_exception &&
                       !s2_mmio &&
                       !s2_isPrefetch  

  // need allocate new entry
  val s2_allocValid = !s2_in.tlbMiss &&
                      !s2_isPrefetch && 
                      !s2_exception && 
                      !s2_mmio  && 
                      !s2_wait_store && 
                      !s2_fast_replay &&
                      !s2_in.replayInfo.cause(LoadReplayCauses.schedError) 

  val s2_dataForwarded = s2_cache_miss && !s2_exception && (s2_fullForward || s2_cache_tag_error)
  // To be removed
  val s2_need_replay_from_rs = WireInit(false.B)
  // s2_cache_replay is quite slow to generate, send it separately to LQ
  val s2_dcache_require_replay = 
      if (EnableFastForward) {
        s2_cache_replay && !s2_fullForward
      } else {
        s2_cache_replay && s2_need_replay_from_rs && !s2_dataForwarded && !s2_isPrefetch && s2_out.miss
      }

  // ld-ld violation require
  io.lsq.ldldVioQuery.req.valid      := s2_valid && s2_allocValid
  io.lsq.ldldVioQuery.req.bits.uop   := s2_in.uop
  io.lsq.ldldVioQuery.req.bits.mask  := s2_in.mask
  io.lsq.ldldVioQuery.req.bits.paddr := s2_in.paddr
  if (EnableFastForward) {
    io.lsq.ldldVioQuery.req.bits.datavalid := Mux(s2_fullForward, true.B, !s2_cache_miss) && !s2_dcache_require_replay
  } else {
    io.lsq.ldldVioQuery.req.bits.datavalid := Mux(s2_fullForward, true.B, !s2_cache_miss)
  }
  // st-ld violation require
  io.lsq.stldVioQuery.req.valid          := s2_valid && s2_allocValid
  io.lsq.stldVioQuery.req.bits.uop       := s2_in.uop
  io.lsq.stldVioQuery.req.bits.mask      := s2_in.mask
  io.lsq.stldVioQuery.req.bits.paddr     := s2_in.paddr
  io.lsq.stldVioQuery.req.bits.datavalid := io.lsq.ldldVioQuery.req.bits.datavalid

  val s2_rarCanAccept = !io.lsq.ldldVioQuery.req.valid || io.lsq.ldldVioQuery.req.ready
  val s2_rawCanAccept = !io.lsq.stldVioQuery.req.valid || io.lsq.stldVioQuery.req.ready
  val s2_rarReject    = !s2_rarCanAccept
  val s2_rawReject    = !s2_rawCanAccept

  // merge forward result
  // lsq has higher priority than sbuffer
  val s2_forwardMask = Wire(Vec(8, Bool()))
  val s2_forwardData = Wire(Vec(8, UInt(8.W)))
  s2_fullForward := ((~s2_forwardMask.asUInt).asUInt & s2_in.mask) === 0.U && !io.lsq.forward.dataInvalid
  // generate XLEN/8 Muxs
  for (i <- 0 until XLEN / 8) {
    s2_forwardMask(i) := io.lsq.forward.forwardMask(i) || io.sbuffer.forwardMask(i)
    s2_forwardData(i) := Mux(io.lsq.forward.forwardMask(i), io.lsq.forward.forwardData(i), io.sbuffer.forwardData(i))
  }

  XSDebug(s2_fire, "[FWD LOAD RESP] pc %x fwd %x(%b) + %x(%b)\n",
    s2_in.uop.cf.pc,
    io.lsq.forward.forwardData.asUInt, io.lsq.forward.forwardMask.asUInt,
    s2_in.forwardData.asUInt, s2_in.forwardMask.asUInt
  )

  // 
  s2_out                     := s2_in
  s2_out.data                := 0.U // data will be generated in load s3
  s2_out.uop.ctrl.fpWen      := s2_in.uop.ctrl.fpWen && !s2_exception
  s2_out.mmio                := s2_mmio
  s2_out.uop.ctrl.flushPipe  := io.fastUop.valid && s2_mmio
  s2_out.uop.cf.exceptionVec := s2_exception_vec
  s2_out.forwardMask         := s2_forwardMask
  s2_out.forwardData         := s2_forwardData
  s2_out.handledByMSHR       := s2_cache_handled
  if (EnableFastForward) {
    s2_out.miss := s2_cache_miss &&
      !s2_exception && 
      !s2_fullForward &&
      !s2_isPrefetch && 
      !s2_mmio
  } else {
    s2_out.miss := s2_cache_miss && 
      !s2_exception &&
      !s2_isPrefetch && 
      !s2_mmio  
  }

  // Generate replay signal caused by:
  // * st-ld violation check
  // * tlb miss
  // * dcache replay
  // * forward data invalid
  // * dcache miss
  s2_out.replayInfo.cause(LoadReplayCauses.waitStore)    := s2_wait_store && !s2_mmio && !s2_isPrefetch
  s2_out.replayInfo.cause(LoadReplayCauses.tlbMiss)      := s2_in.tlbMiss
  s2_out.replayInfo.cause(LoadReplayCauses.schedError)   := (s2_in.replayInfo.cause(LoadReplayCauses.schedError) || s2_sched_err) && !s2_mmio && !s2_isPrefetch
  s2_out.replayInfo.cause(LoadReplayCauses.bankConflict) := io.dcache.s2_bank_conflict && !s2_mmio && !s2_isPrefetch
  s2_out.replayInfo.cause(LoadReplayCauses.dcacheMiss)   := s2_out.miss
  if (EnableFastForward) {
    s2_out.replayInfo.cause(LoadReplayCauses.dcacheReplay) := s2_cache_replay && !s2_isPrefetch && !s2_mmio && !s2_exception && !s2_fullForward
  }else {
    s2_out.replayInfo.cause(LoadReplayCauses.dcacheReplay) := s2_cache_replay && !s2_isPrefetch && !s2_mmio && !s2_exception && !s2_dataForwarded
  }
  s2_out.replayInfo.cause(LoadReplayCauses.forwardFail) := s2_data_invalid && !s2_mmio && !s2_isPrefetch
  s2_out.replayInfo.cause(LoadReplayCauses.rarReject)   := s2_rarReject && !s2_mmio && !s2_isPrefetch && !s2_exception
  s2_out.replayInfo.cause(LoadReplayCauses.rawReject)   := s2_rawReject && !s2_mmio && !s2_isPrefetch && !s2_exception
  s2_out.replayInfo.canForwardFullData                  := s2_dataForwarded
  s2_out.replayInfo.dataInvalidSqIdx                    := io.lsq.forward.dataInvalidSqIdx
  s2_out.replayInfo.addrInvalidSqIdx                    := io.lsq.forward.addrInvalidSqIdx // io.in.bits.uop.sqIdx - io.oracleMDPQuery.resp.distance // io.addrInvalidSqIdx
  s2_out.replayInfo.replayCarry                         := io.dcache.resp.bits.replayCarry
  s2_out.replayInfo.missMSHRId                          := io.dcache.resp.bits.mshr_id
  s2_out.replayInfo.dataInLastBeat                      := s2_in.paddr(log2Up(refillBytes))
  s2_out.replayInfo.debug                               := s2_in.uop.debugInfo

  // if forward fail, replay this inst from fetch
  val debug_forwardFailReplay = s2_forward_fail && !s2_mmio && !s2_isPrefetch && !s2_in.tlbMiss
  // if ld-ld violation is detected, replay from this inst from fetch
  val debug_ldldVioReplay = false.B // s2_ldld_violation && !s2_mmio && !s2_is_prefetch && !s2_in.tlbMiss
  // io.out.bits.uop.ctrl.replayInst := false.B

  io.feedbackFast.valid                 := s2_valid && !s2_in.isLoadReplay && !s2_exception && io.lqReplayFull && s2_out.replayInfo.needReplay() && !s2_out.uop.robIdx.needFlush(io.redirect)
  io.feedbackFast.bits.hit              := false.B 
  io.feedbackFast.bits.flushState       := s2_in.ptwBack
  io.feedbackFast.bits.rsIdx            := s2_in.rsIdx 
  io.feedbackFast.bits.sourceType       := RSFeedbackType.lrqFull
  io.feedbackFast.bits.dataInvalidSqIdx := DontCare

  // fast wakeup
  io.fastUop.valid := RegNext(
    !io.dcache.s1_disable_fast_wakeup && 
    s1_valid &&
    !s1_kill &&
    !io.tlb.resp.bits.fast_miss && 
    !io.lsq.forward.dataInvalidFast
  ) && 
  (s2_valid && s2_cache_hit && s2_out.replayInfo.needReplay())
  io.fastUop.bits := RegNext(s1_out.uop)

  val s1_loadLeftFire = s1_valid && !s1_kill && s2_ready
  val s2_loadValidVec = RegInit(0.U(6.W))
  s2_loadValidVec := 0x0.U(6.W)
  when (s1_loadLeftFire && !s1_out.isHWPrefetch) { s2_loadValidVec := 0x3f.U(6.W) }
  when (s1_kill) { s2_loadValidVec := 0x0.U(6.W) }
  assert(RegNext((s2_valid === s2_loadValidVec(0)) || RegNext(s1_out.isHWPrefetch)))

  // Pipeline
  // --------------------------------------------------------------------------------
  // stage 3
  // --------------------------------------------------------------------------------
  // writeback and update load queue
  val s3_valid = RegNext(s2_valid) && !RegNext(s2_out.uop.robIdx.needFlush(io.redirect))
  val s3_in    = RegEnable(s2_out, s2_fire)
  val s3_out   = Wire(new LqWriteBundle) 
  val s3_kill  = s3_out.uop.robIdx.needFlush(io.redirect)
  s3_ready := !s3_valid || s3_kill || io.loadOut.ready

  val s3_fast_replay = Wire(Bool())
  io.lsq.loadIn.valid := s3_valid && (!s3_fast_replay || !io.fastReplayOut.ready)
  io.lsq.loadIn.bits := s3_in

  // s3 load fast replay
  io.fastReplayOut.valid := s3_valid && s3_fast_replay && !s3_in.uop.robIdx.needFlush(io.redirect)
  io.fastReplayOut.bits := s3_in

  /* <------- DANGEROUS: Don't change sequence here ! -------> */

  val s3_loadValidVec = Reg(UInt(6.W))
  s3_loadValidVec := s2_loadValidVec
  io.lsq.loadIn.bits.lqDataWenDup := s3_loadValidVec.asBools
  io.lsq.loadIn.bits.replacementUpdated := io.dcache.resp.bits.replacementUpdated

  val s3_dcacheRequireReplay = RegNext(s2_dcache_require_replay)
  val s3_delayedLoadError =  
    if (EnableAccurateLoadError) {
      io.dcache.resp.bits.error_delayed && RegNext(io.csrCtrl.cache_error_enable)
    } else {
      WireInit(false.B)
    }
  val s3_canReplayFromFetch = RegNext(s2_canReplayFromFetch) 
  io.s3_delayedLoadError := s3_delayedLoadError
  io.lsq.loadIn.bits.dcacheRequireReplay := s3_dcacheRequireReplay

  val s3_vpMatchInvalid = RegNext(io.lsq.forward.matchInvalid || io.sbuffer.matchInvalid)
  val s3_ldld_replayFromFetch =  
      io.lsq.ldldVioQuery.resp.valid && 
      io.lsq.ldldVioQuery.resp.bits.replayFromFetch && 
      RegNext(io.csrCtrl.ldld_vio_check_enable)

  val s3_replayInfo = s3_in.replayInfo
  val s3_replayInst = s3_vpMatchInvalid || s3_ldld_replayFromFetch
  val s3_selReplayCause = PriorityEncoderOH(s3_replayInfo.cause.asUInt)
  val s3_forceReplay = s3_selReplayCause(LoadReplayCauses.schedError) || 
                       s3_selReplayCause(LoadReplayCauses.tlbMiss) ||
                       s3_selReplayCause(LoadReplayCauses.waitStore)

  val s3_exception = ExceptionNO.selectByFu(s3_in.uop.cf.exceptionVec, lduCfg).asUInt.orR
  when ((s3_exception || s3_delayedLoadError || s3_replayInst) && !s3_forceReplay) { 
    io.lsq.loadIn.bits.replayInfo.cause := 0.U.asTypeOf(s3_replayInfo.cause.cloneType)
  } .otherwise {
    io.lsq.loadIn.bits.replayInfo.cause := VecInit(s3_selReplayCause.asBools)
  }

  // Int load, if hit, will be writebacked at s2
  val hitLoadOut = Wire(Valid(new ExuOutput))
  hitLoadOut.valid                := s3_valid && !io.lsq.loadIn.bits.replayInfo.needReplay() && !s3_in.mmio
  hitLoadOut.bits.uop             := s3_in.uop
  hitLoadOut.bits.uop.cf.exceptionVec(loadAccessFault) := s3_delayedLoadError && !s3_in.tlbMiss  || 
                                                          s3_in.uop.cf.exceptionVec(loadAccessFault) 
  hitLoadOut.bits.uop.ctrl.replayInst := s3_replayInst
  hitLoadOut.bits.data            := s3_in.data
  hitLoadOut.bits.redirectValid   := false.B
  hitLoadOut.bits.redirect        := DontCare
  hitLoadOut.bits.debug.isMMIO    := s3_in.mmio
  hitLoadOut.bits.debug.isPerfCnt := false.B
  hitLoadOut.bits.debug.paddr     := s3_in.paddr
  hitLoadOut.bits.debug.vaddr     := s3_in.vaddr
  hitLoadOut.bits.fflags          := DontCare

  when (s3_forceReplay) {
    hitLoadOut.bits.uop.cf.exceptionVec := 0.U.asTypeOf(s3_in.uop.cf.exceptionVec.cloneType)
  }

  /* <------- DANGEROUS: Don't change sequence here ! -------> */
  
  io.lsq.loadIn.bits.uop := hitLoadOut.bits.uop

  val s3_needRelease = s3_exception || io.lsq.loadIn.bits.replayInfo.needReplay()
  io.lsq.ldldVioQuery.preReq := s1_valid
  io.lsq.ldldVioQuery.release := s3_needRelease
  io.lsq.stldVioQuery.preReq := s1_valid
  io.lsq.stldVioQuery.release := s3_needRelease

  // feedback slow
  s3_fast_replay := (RegNext(s2_fast_replay) || 
                    (s3_in.replayInfo.cause(LoadReplayCauses.dcacheMiss) && io.l2Hint.valid && io.l2Hint.bits.sourceId === s3_in.replayInfo.missMSHRId)) && 
                    !s3_exception
  val s3_need_feedback = !s3_in.isLoadReplay && !(s3_fast_replay && io.fastReplayOut.ready)

  //
  io.feedbackSlow.valid                 := s3_valid && !s3_in.uop.robIdx.needFlush(io.redirect) && s3_need_feedback 
  io.feedbackSlow.bits.hit              := !io.lsq.loadIn.bits.replayInfo.needReplay() || io.lsq.loadIn.ready
  io.feedbackSlow.bits.flushState       := s3_in.ptwBack
  io.feedbackSlow.bits.rsIdx            := s3_in.rsIdx
  io.feedbackSlow.bits.sourceType       := RSFeedbackType.lrqFull
  io.feedbackSlow.bits.dataInvalidSqIdx := DontCare

  val s3_loadWbMeta = Mux(hitLoadOut.valid, hitLoadOut.bits, io.lsq.loadOut.bits)

  // data from load queue refill
  val s3_loadDataFromLQ = io.lsq.ldRawData
  val s3_rdataLQ = s3_loadDataFromLQ.mergedData()
  val s3_rdataSelLQ = LookupTree(s3_loadDataFromLQ.addrOffset, List(
    "b000".U -> s3_rdataLQ(63,  0),
    "b001".U -> s3_rdataLQ(63,  8),
    "b010".U -> s3_rdataLQ(63, 16),
    "b011".U -> s3_rdataLQ(63, 24),
    "b100".U -> s3_rdataLQ(63, 32),
    "b101".U -> s3_rdataLQ(63, 40),
    "b110".U -> s3_rdataLQ(63, 48),
    "b111".U -> s3_rdataLQ(63, 56)
  ))
  val s3_rdataPartialLoadLQ = rdataHelper(s3_loadDataFromLQ.uop, s3_rdataSelLQ)

  // data from dcache hit
  val s3_loadDataFromDcache = Wire(new LoadDataFromDcacheBundle)
  s3_loadDataFromDcache.respDcacheData       := io.dcache.resp.bits.data_delayed
  s3_loadDataFromDcache.forwardMask          := RegEnable(s2_forwardMask, s2_valid)
  s3_loadDataFromDcache.forwardData          := RegEnable(s2_forwardData, s2_valid)
  s3_loadDataFromDcache.uop                  := RegEnable(s2_out.uop, s2_valid)
  s3_loadDataFromDcache.addrOffset           := RegEnable(s2_out.paddr(2, 0), s2_valid)
  s3_loadDataFromDcache.forward_D            := RegEnable(s2_forward_D, s2_valid)
  s3_loadDataFromDcache.forwardData_D        := RegEnable(s2_forwardData_D, s2_valid)
  s3_loadDataFromDcache.forward_mshr         := RegEnable(s2_forward_mshr, s2_valid)
  s3_loadDataFromDcache.forwardData_mshr     := RegEnable(s2_forwardData_mshr, s2_valid)
  s3_loadDataFromDcache.forward_result_valid := RegEnable(s2_forward_result_valid, s2_valid)

  val s3_rdataDcache = s3_loadDataFromDcache.mergedData()
  val s3_rdataSelDcache = LookupTree(s3_loadDataFromDcache.addrOffset, List(
    "b000".U -> s3_rdataDcache(63,  0),
    "b001".U -> s3_rdataDcache(63,  8),
    "b010".U -> s3_rdataDcache(63, 16),
    "b011".U -> s3_rdataDcache(63, 24),
    "b100".U -> s3_rdataDcache(63, 32),
    "b101".U -> s3_rdataDcache(63, 40),
    "b110".U -> s3_rdataDcache(63, 48),
    "b111".U -> s3_rdataDcache(63, 56)
  ))
  val s3_rdataPartialLoadDcache = rdataHelper(s3_loadDataFromDcache.uop, s3_rdataSelDcache)

  // FIXME: add 1 cycle delay ?
  io.loadOut.bits := s3_loadWbMeta
  io.loadOut.bits.data := Mux(hitLoadOut.valid, s3_rdataPartialLoadDcache, s3_rdataPartialLoadLQ)
  io.loadOut.valid := hitLoadOut.valid && !hitLoadOut.bits.uop.robIdx.needFlush(io.redirect) ||
                    io.lsq.loadOut.valid && !io.lsq.loadOut.bits.uop.robIdx.needFlush(io.redirect) && !hitLoadOut.valid
  
  io.lsq.loadOut.ready := !hitLoadOut.valid

  // fast load to load forward
  io.fastpathOut.valid := hitLoadOut.valid // for debug only
  io.fastpathOut.data := s3_loadDataFromDcache.mergedData() // fastpath is for ld only

   // trigger
  val lastValidData = RegNext(RegEnable(io.loadOut.bits.data, io.loadOut.fire))
  val hitLoadAddrTriggerHitVec = Wire(Vec(3, Bool()))
  val lqLoadAddrTriggerHitVec = io.lsq.trigger.lqLoadAddrTriggerHitVec
  (0 until 3).map{i => {
    val tdata2 = RegNext(io.trigger(i).tdata2)
    val matchType = RegNext(io.trigger(i).matchType)
    val tEnable = RegNext(io.trigger(i).tEnable)

    hitLoadAddrTriggerHitVec(i) := TriggerCmp(RegNext(s2_out.vaddr), tdata2, matchType, tEnable)
    io.trigger(i).addrHit       := Mux(hitLoadOut.valid, hitLoadAddrTriggerHitVec(i), lqLoadAddrTriggerHitVec(i))
    io.trigger(i).lastDataHit   := TriggerCmp(lastValidData, tdata2, matchType, tEnable)
  }}
  io.lsq.trigger.hitLoadAddrTriggerHitVec := hitLoadAddrTriggerHitVec

  // FIXME: please move this part to LoadQueueReplay 
  io.debug_ls := DontCare

  // Topdown
  io.lsTopdownInfo.s1.robIdx      := s1_in.uop.robIdx.value
  io.lsTopdownInfo.s1.vaddr_valid := s1_valid && s1_in.hasROBEntry
  io.lsTopdownInfo.s1.vaddr_bits  := s1_in.vaddr
  io.lsTopdownInfo.s2.robIdx      := s2_in.uop.robIdx.value
  io.lsTopdownInfo.s2.paddr_valid := s2_fire && s2_in.hasROBEntry && !s2_in.tlbMiss
  io.lsTopdownInfo.s2.paddr_bits  := s2_in.paddr  

  // perf cnt
  XSPerfAccumulate("in_valid",                  io.loadIn.valid)
  XSPerfAccumulate("in_block",                  io.loadIn.valid && !io.loadIn.fire)
  XSPerfAccumulate("in_fire_first_issue",       s0_valid && s0_isFirstIssue)
  XSPerfAccumulate("lsq_fire_first_issue",      lfsrc_loadReplay_select)
  XSPerfAccumulate("ldu_fire_first_issue",      lfsrc_intloadFirstIssue_select && s0_isFirstIssue)
  XSPerfAccumulate("fast_replay_issue",         lfsrc_loadFastReplay_select)
  XSPerfAccumulate("stall_dcache",              s0_fire && !io.dcache.req.ready)
  XSPerfAccumulate("addr_spec_success",         s0_fire && s0_vaddr(VAddrBits-1, 12) === io.loadIn.bits.src(0)(VAddrBits-1, 12))
  XSPerfAccumulate("addr_spec_failed",          s0_fire && s0_vaddr(VAddrBits-1, 12) =/= io.loadIn.bits.src(0)(VAddrBits-1, 12))
  XSPerfAccumulate("addr_spec_success_once",    s0_fire && s0_vaddr(VAddrBits-1, 12) === io.loadIn.bits.src(0)(VAddrBits-1, 12) && s0_isFirstIssue)
  XSPerfAccumulate("addr_spec_failed_once",     s0_fire && s0_vaddr(VAddrBits-1, 12) =/= io.loadIn.bits.src(0)(VAddrBits-1, 12) && s0_isFirstIssue)
  XSPerfAccumulate("forward_tlDchannel",        s0_out.forward_tlDchannel)
  XSPerfAccumulate("hardware_prefetch_fire",    s0_fire && lfsrc_hwprefetch_select)
  XSPerfAccumulate("software_prefetch_fire",    s0_fire && s0_isPrefetch && lfsrc_intloadFirstIssue_select)
  XSPerfAccumulate("hardware_prefetch_blocked", io.prefetch_req.valid && !lfsrc_hwprefetch_select)
  XSPerfAccumulate("hardware_prefetch_total",   io.prefetch_req.valid)
  XSPerfAccumulate("stall_out",                 s0_valid && !s0_can_go)

  XSPerfAccumulate("in_valid",                  s1_valid)
  XSPerfAccumulate("in_fire",                   s1_fire)
  XSPerfAccumulate("in_fire_first_issue",       s1_fire && s1_in.isFirstIssue)
  XSPerfAccumulate("tlb_miss",                  s1_fire && s1_tlb_miss)
  XSPerfAccumulate("tlb_miss_first_issue",      s1_fire && s1_tlb_miss && s1_in.isFirstIssue)
  XSPerfAccumulate("stall_out",                 s1_valid && !s1_can_go)

  XSPerfAccumulate("in_valid",                  s2_valid)
  XSPerfAccumulate("in_fire",                   s2_fire)
  XSPerfAccumulate("in_fire_first_issue",       s2_fire && s2_in.isFirstIssue)
  XSPerfAccumulate("dcache_miss",               s2_fire && s2_cache_miss)
  XSPerfAccumulate("dcache_miss_first_issue",   s2_fire && s2_cache_miss && s2_in.isFirstIssue)
  XSPerfAccumulate("full_forward",              s2_valid && s2_fullForward)
  XSPerfAccumulate("dcache_miss_full_forward",  s2_valid && s2_cache_miss && s2_fullForward)
  XSPerfAccumulate("stall_out",                 s2_valid && !s2_can_go)
  XSPerfAccumulate("prefetch",                  s2_fire && s2_isPrefetch)
  XSPerfAccumulate("prefetch_ignored",          s2_fire && s2_isPrefetch && s2_cache_replay) // ignore prefetch for mshr full / miss req port conflict
  XSPerfAccumulate("prefetch_miss",             s2_fire && s2_isPrefetch && s2_cache_miss) // prefetch req miss in l1 
  XSPerfAccumulate("prefetch_hit",              s2_fire && s2_isPrefetch && !s2_cache_miss) // prefetch req hit in l1 
  XSPerfAccumulate("prefetch_accept",           s2_fire && s2_isPrefetch && s2_cache_miss && !s2_cache_replay) // prefetch a missed line in l1, and l1 accepted it
  XSPerfAccumulate("successfully_forward_channel_D", s2_forward_D && s2_forward_result_valid)
  XSPerfAccumulate("successfully_forward_mshr",      s2_forward_mshr && s2_forward_result_valid)

  XSPerfAccumulate("load_to_load_forward",                      s1_tryPointerChasing && !s1_pointerChasingCanceled)
  XSPerfAccumulate("load_to_load_forward_try",                  s1_tryPointerChasing)
  XSPerfAccumulate("load_to_load_forward_fail",                 s1_cancelPointerChasing)
  XSPerfAccumulate("load_to_load_forward_fail_cancelled",       s1_cancelPointerChasing && s1_pointerChasingCancelled)
  XSPerfAccumulate("load_to_load_forward_fail_wakeup_mismatch", s1_cancelPointerChasing && !s1_pointerChasingCancelled && s1_notFastMatch)
  XSPerfAccumulate("load_to_load_forward_fail_op_not_ld",       s1_cancelPointerChasing && !s1_pointerChasingCancelled && !s1_notFastMatch && s1_fuOpTypeIsNotLd)
  XSPerfAccumulate("load_to_load_forward_fail_addr_align",      s1_cancelPointerChasing && !s1_pointerChasingCancelled && !s1_notFastMatch && !s1_fuOpTypeIsNotLd && s1_addressNotAligned)
  XSPerfAccumulate("load_to_load_forward_fail_set_mismatch",    s1_cancelPointerChasing && !s1_pointerChasingCancelled && !s1_notFastMatch && !s1_fuOpTypeIsNotLd && !s1_addressNotAligned && s1_addressMisMatch)

  // bug lyq: some signals in perfEvents are no longer suitable for the current MemBlock design
  // hardware performance counter
  val perfEvents = Seq(
    ("load_s0_in_fire         ", s0_fire                                                        ),
    ("load_to_load_forward    ", s1_valid && s1_tryPointerChasing && !s1_pointerChasingCanceled ),
    ("stall_dcache            ", s0_valid && s0_can_go && !io.dcache.req.ready                  ),
    ("load_s1_in_fire         ", s0_fire                                                        ),
    ("load_s1_tlb_miss        ", s1_valid && io.tlb.resp.bits.miss                              ),
    ("load_s2_in_fire         ", s1_fire                                                        ),
    ("load_s2_dcache_miss     ", s2_valid && io.dcache.resp.bits.miss                           ),
  )
  generatePerfEvent()

  when(io.loadOut.fire){
    XSDebug("loadOut %x\n", io.loadOut.bits.uop.cf.pc)
  }
  // end
}