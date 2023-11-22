package xiangshan.backend.issue

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utility.HasCircularQueuePtrHelper
import utils.{MathUtils, OptionWrapper, XSError}
import xiangshan._
import xiangshan.backend.Bundles._
import xiangshan.backend.datapath.DataConfig.VAddrData
import xiangshan.backend.datapath.DataSource
import xiangshan.backend.fu.FuType
import xiangshan.backend.fu.vector.Utils.NOnes
import xiangshan.backend.rob.RobPtr
import xiangshan.mem.{MemWaitUpdateReq, SqPtr, LqPtr}

class StatusMemPart(implicit p:Parameters, params: IssueBlockParams) extends Bundle {
  val waitForSqIdx = new SqPtr   // generated by store data valid check
  val waitForRobIdx = new RobPtr // generated by store set
  val waitForStd = Bool()
  val strictWait = Bool()
  val sqIdx = new SqPtr
}

class StatusVecMemPart(implicit p:Parameters, params: IssueBlockParams) extends Bundle {
  val sqIdx = new SqPtr
  val lqIdx = new LqPtr
}

class Status(implicit p:Parameters, params: IssueBlockParams) extends XSBundle {
  val srcState = Vec(params.numRegSrc, SrcState())

  val psrc = Vec(params.numRegSrc, UInt(params.rdPregIdxWidth.W))
  val srcType = Vec(params.numRegSrc, SrcType())
  val fuType = FuType()
  val robIdx = new RobPtr
  val uopIdx = OptionWrapper(params.isVecMemIQ, UopIdx())
  val issued = Bool()           // for predict issue
  val firstIssue = Bool()
  val blocked = Bool()          // for some block reason
  // read reg or get data from bypass network
  val dataSources = Vec(params.numRegSrc, DataSource())
  // if waked up by iq, set when waked up by iq
  val srcWakeUpL1ExuOH = OptionWrapper(params.hasIQWakeUp, Vec(params.numRegSrc, ExuOH()))
  // src timer, used by cancel signal. It increases every cycle after wakeup src inst issued.
  val srcTimer = OptionWrapper(params.hasIQWakeUp, Vec(params.numRegSrc, UInt(3.W)))
  val issueTimer = UInt(2.W)
  val deqPortIdx = UInt(1.W)
  val srcLoadDependency = OptionWrapper(params.hasIQWakeUp, Vec(params.numRegSrc, Vec(LoadPipelineWidth, UInt(3.W))))

  // mem only
  val mem = if (params.isMemAddrIQ) Some(new StatusMemPart) else None

  // vector mem only
  val vecMem = if (params.isVecMemIQ) Some(new StatusVecMemPart) else None

  def srcReady: Bool = {
    VecInit(srcState.map(SrcState.isReady)).asUInt.andR
  }

  def canIssue: Bool = {
    srcReady && !issued && !blocked
  }

  def mergedLoadDependency = {
    srcLoadDependency.map(_.map(_.toSeq).reduce({
      case (l: Vec[UInt], r: Vec[UInt]) => VecInit(l.zip(r).map(x => x._1 | x._2))
    }: (Vec[UInt], Vec[UInt]) => Vec[UInt]))
  }
}

class EntryDeqRespBundle(implicit p:Parameters, params: IssueBlockParams) extends Bundle {
  val robIdx = new RobPtr
  val uopIdx = UopIdx()
  val respType = RSFeedbackType()   // update credit if needs replay
  val dataInvalidSqIdx = new SqPtr
  val rfWen = Bool()
  val fuType = FuType()
}

class EntryBundle(implicit p: Parameters, params: IssueBlockParams) extends XSBundle {
  val status = new Status()
  val imm = OptionWrapper(params.needImm, UInt((params.deqImmTypesMaxLen).W))
  val payload = new DynInst()
}

class DeqBundle(implicit p:Parameters, params: IssueBlockParams) extends XSBundle {
  //input
  val enqEntryOldestSel = Flipped(ValidIO(UInt(params.numEnq.W)))
  val othersEntryOldestSel = Flipped(ValidIO(UInt((params.numEntries - params.numEnq).W)))
  val subDeqRequest = OptionWrapper(params.deqFuSame, Input(UInt(params.numEntries.W)))
  val subDeqSelOH = OptionWrapper(params.deqFuSame, Input(UInt(params.numEntries.W)))
  val deqReady = Input(Bool())
  val deqSelOH = Flipped(ValidIO(UInt(params.numEntries.W)))
  //output
  val isFirstIssue = Output(Bool())
  val deqEntry = ValidIO(new EntryBundle)
}

class EntriesIO(implicit p: Parameters, params: IssueBlockParams) extends XSBundle {
  val flush = Flipped(ValidIO(new Redirect))
  // status
  val valid = Output(UInt(params.numEntries.W))
  val canIssue = Output(UInt(params.numEntries.W))
  val clear = Output(UInt(params.numEntries.W))
  val fuType = Output(Vec(params.numEntries, FuType()))
  val dataSources = Output(Vec(params.numEntries, Vec(params.numRegSrc, DataSource())))
  val srcWakeUpL1ExuOH = OptionWrapper(params.hasIQWakeUp, Output(Vec(params.numEntries, Vec(params.numRegSrc, ExuOH()))))
  val srcTimer = OptionWrapper(params.hasIQWakeUp, Output(Vec(params.numEntries, Vec(params.numRegSrc, UInt(3.W)))))
  val robIdx = OptionWrapper(params.isVecMemIQ, Output(Vec(params.numEntries, new RobPtr)))
  val uopIdx = OptionWrapper(params.isVecMemIQ, Output(Vec(params.numEntries, UopIdx())))
  //enq
  val enq = Vec(params.numEnq, Flipped(ValidIO(new EntryBundle)))
  // wakeup
  val wakeUpFromWB: MixedVec[ValidIO[IssueQueueWBWakeUpBundle]] = Flipped(params.genWBWakeUpSinkValidBundle)
  val wakeUpFromIQ: MixedVec[ValidIO[IssueQueueIQWakeUpBundle]] = Flipped(params.genIQWakeUpSinkValidBundle)
  val og0Cancel = Input(ExuOH(backendParams.numExu))
  val og1Cancel = Input(ExuOH(backendParams.numExu))
  val ldCancel = Vec(backendParams.LduCnt + backendParams.HyuCnt, Flipped(new LoadCancelIO))
  //deq
  val deq = Vec(params.numDeq, new DeqBundle)
  val og0Resp = Vec(params.numDeq, Flipped(ValidIO(new EntryDeqRespBundle)))
  val og1Resp = Vec(params.numDeq, Flipped(ValidIO(new EntryDeqRespBundle)))
  val finalIssueResp = OptionWrapper(params.LdExuCnt > 0, Vec(params.LdExuCnt, Flipped(ValidIO(new EntryDeqRespBundle))))
  val memAddrIssueResp = OptionWrapper(params.LdExuCnt > 0, Vec(params.LdExuCnt, Flipped(ValidIO(new EntryDeqRespBundle))))
  val transEntryDeqVec = Vec(params.numEnq, ValidIO(new EntryBundle))
  val transSelVec = Output(Vec(params.numEnq, UInt((params.numEntries-params.numEnq).W)))
  val cancelDeqVec = Output(Vec(params.numDeq, Bool()))


  val rsFeedback = Output(Vec(5, Bool()))
  // mem only
  val fromMem = if (params.isMemAddrIQ) Some(new Bundle {
    val stIssuePtr = Input(new SqPtr)
    val memWaitUpdateReq = Flipped(new MemWaitUpdateReq)
    val slowResp = Vec(params.numDeq, Flipped(ValidIO(new EntryDeqRespBundle)))
    val fastResp = Vec(params.numDeq, Flipped(ValidIO(new EntryDeqRespBundle)))
  }) else None

  // vector mem only
  val fromLsq = OptionWrapper(params.isVecMemIQ, new Bundle {
    val sqDeqPtr = Input(new SqPtr)
    val lqDeqPtr = Input(new LqPtr)
  })

  // debug
  val cancel = OptionWrapper(params.hasIQWakeUp, Output(Vec(params.numEntries, Bool())))

  def wakeup = wakeUpFromWB ++ wakeUpFromIQ
}

class Entries(implicit p: Parameters, params: IssueBlockParams) extends XSModule {
  private val EnqEntryNum = params.numEnq
  private val OthersEntryNum = params.numEntries - params.numEnq
  val io = IO(new EntriesIO)

  // only memAddrIQ use it
  val memEtyResps: MixedVec[ValidIO[EntryDeqRespBundle]] = {
    if (params.isLdAddrIQ && !params.isStAddrIQ)
      MixedVecInit(io.og0Resp ++ io.og1Resp ++ io.memAddrIssueResp.get ++ io.finalIssueResp.get)
    else if (params.isLdAddrIQ && params.isStAddrIQ || params.isHyAddrIQ)
      MixedVecInit(io.og0Resp ++ io.og1Resp ++ io.memAddrIssueResp.get ++ io.finalIssueResp.get ++ io.fromMem.get.fastResp ++ io.fromMem.get.slowResp)
    else if (params.isMemAddrIQ)
      MixedVecInit(io.og0Resp ++ io.og1Resp ++ io.fromMem.get.fastResp ++ io.fromMem.get.slowResp)
    else MixedVecInit(Seq())
  }

  val resps: Vec[Vec[ValidIO[EntryDeqRespBundle]]] = VecInit(io.og0Resp, io.og1Resp, 0.U.asTypeOf(io.og0Resp), 0.U.asTypeOf(io.og0Resp))

  //Module
  val enqEntries = Seq.fill(EnqEntryNum)(Module(EnqEntry(p, params)))
  val othersEntries = Seq.fill(OthersEntryNum)(Module(OthersEntry(p, params)))
  val transPolicy = Module(new EnqPolicy)

  //Wire
  val deqSelVec = Wire(Vec(params.numEntries, Bool()))
  val transSelVec = Wire(Vec(EnqEntryNum, Vec(OthersEntryNum, Bool())))
  val issueRespVec = Wire(Vec(params.numEntries, ValidIO(new EntryDeqRespBundle)))
  val transEntryDeqVec = Wire(Vec(EnqEntryNum, ValidIO(new EntryBundle)))
  val transEntryEnqVec = Wire(Vec(OthersEntryNum, ValidIO(new EntryBundle)))
  val entries = Wire(Vec(params.numEntries, ValidIO(new EntryBundle)))

  val validVec = Wire(Vec(params.numEntries, Bool()))
  val canIssueVec = Wire(Vec(params.numEntries, Bool()))
  val clearVec = Wire(Vec(params.numEntries, Bool()))
  val fuTypeVec = Wire(Vec(params.numEntries, FuType()))
  val dataSourceVec = Wire(Vec(params.numEntries, Vec(params.numRegSrc, DataSource())))
  val srcWakeUpL1ExuOHVec = OptionWrapper(params.hasIQWakeUp, Wire(Vec(params.numEntries, Vec(params.numRegSrc, ExuOH()))))
  val srcTimerVec = OptionWrapper(params.hasIQWakeUp, Wire(Vec(params.numEntries, Vec(params.numRegSrc, UInt(3.W)))))
  val cancelByOg0Vec = OptionWrapper(params.hasIQWakeUp, Wire(Vec(params.numEntries, Bool())))
  val isFirstIssueVec = Wire(Vec(params.numEntries, Bool()))
  val robIdxVec = Wire(Vec(params.numEntries, new RobPtr))
  val issueTimerVec = Wire(Vec(params.numEntries, UInt(2.W)))
  val deqPortIdxWriteVec = Wire(Vec(params.numEntries, UInt(1.W)))
  val deqPortIdxReadVec = Wire(Vec(params.numEntries, UInt(1.W)))
  val cancelVec = OptionWrapper(params.hasIQWakeUp, Wire(Vec(params.numEntries, Bool())))
  val uopIdxVec = OptionWrapper(params.isVecMemIQ, Wire(Vec(params.numEntries, UopIdx())))

  io.transEntryDeqVec := transEntryDeqVec

  //enqEntries
  enqEntries.zipWithIndex.foreach { case (enqEntry, entryIdx) =>
    enqEntry.io.enq := io.enq(entryIdx)
    enqEntry.io.flush := io.flush
    enqEntry.io.wakeUpFromWB := io.wakeUpFromWB
    enqEntry.io.wakeUpFromIQ := io.wakeUpFromIQ
    enqEntry.io.og0Cancel := io.og0Cancel
    enqEntry.io.og1Cancel := io.og1Cancel
    enqEntry.io.ldCancel := io.ldCancel
    enqEntry.io.enqDelayWakeUpFromWB := RegNext(io.wakeUpFromWB)
    enqEntry.io.enqDelayWakeUpFromIQ := RegNext(io.wakeUpFromIQ)
    enqEntry.io.enqDelayOg0Cancel := RegNext(io.og0Cancel)
    enqEntry.io.enqDelayLdCancel := RegNext(io.ldCancel)
    enqEntry.io.deqSel := deqSelVec(entryIdx)
    enqEntry.io.deqPortIdxWrite := deqPortIdxWriteVec(entryIdx)
    enqEntry.io.transSel := transSelVec(entryIdx).asUInt.orR
    enqEntry.io.issueResp := issueRespVec(entryIdx)
    validVec(entryIdx) := enqEntry.io.valid
    canIssueVec(entryIdx) := enqEntry.io.canIssue
    clearVec(entryIdx) := enqEntry.io.clear
    fuTypeVec(entryIdx) := enqEntry.io.fuType
    dataSourceVec(entryIdx) := enqEntry.io.dataSource
    robIdxVec(entryIdx) := enqEntry.io.robIdx
    issueTimerVec(entryIdx) := enqEntry.io.issueTimerRead
    deqPortIdxReadVec(entryIdx) := enqEntry.io.deqPortIdxRead
    if (params.hasIQWakeUp) {
      srcWakeUpL1ExuOHVec.get(entryIdx) := enqEntry.io.srcWakeUpL1ExuOH.get
      srcTimerVec.get(entryIdx) := enqEntry.io.srcTimer.get
      cancelVec.get(entryIdx) := enqEntry.io.cancel.get
    }
    transEntryDeqVec(entryIdx) := enqEntry.io.transEntry
    isFirstIssueVec(entryIdx) := enqEntry.io.isFirstIssue
    entries(entryIdx) := enqEntry.io.entry
    //for mem
    if (params.isMemAddrIQ) {
      enqEntry.io.fromMem.get.stIssuePtr := io.fromMem.get.stIssuePtr
      enqEntry.io.fromMem.get.memWaitUpdateReq := io.fromMem.get.memWaitUpdateReq
    }
    if (params.isVecMemIQ) {
      uopIdxVec.get(entryIdx) := enqEntry.io.uopIdx.get
      enqEntry.io.fromLsq.get.sqDeqPtr := io.fromLsq.get.sqDeqPtr
      enqEntry.io.fromLsq.get.lqDeqPtr := io.fromLsq.get.lqDeqPtr
    }
  }
  //othersEntries
  othersEntries.zipWithIndex.foreach { case (othersEntry, entryIdx) =>
    othersEntry.io.enq := transEntryEnqVec(entryIdx)
    othersEntry.io.flush := io.flush
    othersEntry.io.wakeUpFromWB := io.wakeUpFromWB
    othersEntry.io.wakeUpFromIQ := io.wakeUpFromIQ
    othersEntry.io.og0Cancel := io.og0Cancel
    othersEntry.io.og1Cancel := io.og1Cancel
    othersEntry.io.ldCancel := io.ldCancel
    othersEntry.io.deqSel := deqSelVec(entryIdx + EnqEntryNum)
    othersEntry.io.deqPortIdxWrite := deqPortIdxWriteVec(entryIdx + EnqEntryNum)
    othersEntry.io.transSel := transSelVec.map(x => x(entryIdx)).reduce(_ | _)
    othersEntry.io.issueResp := issueRespVec(entryIdx + EnqEntryNum)
    validVec(entryIdx + EnqEntryNum) := othersEntry.io.valid
    canIssueVec(entryIdx + EnqEntryNum) := othersEntry.io.canIssue
    clearVec(entryIdx + EnqEntryNum) := othersEntry.io.clear
    fuTypeVec(entryIdx + EnqEntryNum) := othersEntry.io.fuType
    dataSourceVec(entryIdx + EnqEntryNum) := othersEntry.io.dataSource
    robIdxVec(entryIdx + EnqEntryNum) := othersEntry.io.robIdx
    issueTimerVec(entryIdx + EnqEntryNum) := othersEntry.io.issueTimerRead
    deqPortIdxReadVec(entryIdx + EnqEntryNum) := othersEntry.io.deqPortIdxRead
    if (params.hasIQWakeUp) {
      srcWakeUpL1ExuOHVec.get(entryIdx + EnqEntryNum) := othersEntry.io.srcWakeUpL1ExuOH.get
      srcTimerVec.get(entryIdx + EnqEntryNum) := othersEntry.io.srcTimer.get
      cancelVec.get(entryIdx + EnqEntryNum) := othersEntry.io.cancel.get
    }
    isFirstIssueVec(entryIdx + EnqEntryNum) := othersEntry.io.isFirstIssue
    entries(entryIdx + EnqEntryNum) := othersEntry.io.entry
    //for mem
    if (params.isMemAddrIQ) {
      othersEntry.io.fromMem.get.stIssuePtr := io.fromMem.get.stIssuePtr
      othersEntry.io.fromMem.get.memWaitUpdateReq := io.fromMem.get.memWaitUpdateReq
    }
    if (params.isVecMemIQ) {
      uopIdxVec.get(entryIdx + EnqEntryNum) := othersEntry.io.uopIdx.get
      othersEntry.io.fromLsq.get.sqDeqPtr := io.fromLsq.get.sqDeqPtr
      othersEntry.io.fromLsq.get.lqDeqPtr := io.fromLsq.get.lqDeqPtr
    }
  }


  deqSelVec.zip(deqPortIdxWriteVec).zipWithIndex.foreach { case ((deqSel, deqPortIdxWrite), i) =>
    val deqVec = io.deq.map(x => x.deqSelOH.valid && x.deqSelOH.bits(i) && x.deqReady)
    deqPortIdxWrite := OHToUInt(deqVec)
    deqSel := deqVec.reduce(_ | _)
  }


  //transPolicy
  transPolicy.io.valid := VecInit(validVec.slice(EnqEntryNum, params.numEntries)).asUInt
  transSelVec.zip(transPolicy.io.enqSelOHVec).foreach { case (selBools, selOH) =>
    selBools.zipWithIndex.foreach { case (selBool, i) =>
      selBool := transPolicy.io.enqSelOHVec.map(_.valid).reduce(_ & _) && selOH.bits(i)
    }
  }

  //transEntryEnq
  transEntryEnqVec.zipWithIndex.foreach { case (transEntryEnq, othersIdx) =>
    val transEnqHit = transSelVec.map(x => x(othersIdx))
    transEntryEnq := Mux1H(transEnqHit, transEntryDeqVec)
  }
  if(backendParams.debugEn) {
    dontTouch(transEntryEnqVec)
  }

  //issueRespVec
  if (params.isVecMemIQ) {
    // vector memory IQ
    issueRespVec.zip(robIdxVec).zip(uopIdxVec.get).foreach { case ((issueResp, robIdx), uopIdx) =>
      val hitRespsVec = VecInit(resps.flatten.map(x =>
        x.valid && x.bits.robIdx === robIdx && x.bits.uopIdx === uopIdx
      ))
      issueResp.valid := hitRespsVec.reduce(_ | _)
      issueResp.bits := Mux1H(hitRespsVec, resps.flatten.map(_.bits))
    }
  } else if (params.isMemAddrIQ) {
    // scalar memory IQ
    issueRespVec.zip(robIdxVec).foreach { case (issueResp, robIdx) =>
      val hitRespsVec = VecInit(memEtyResps.map(x => x.valid && (x.bits.robIdx === robIdx)).toSeq)
      issueResp.valid := hitRespsVec.reduce(_ | _)
      issueResp.bits := Mux1H(hitRespsVec, memEtyResps.map(_.bits).toSeq)
    }
  }
  else {
    issueRespVec.zip(issueTimerVec).zip(deqPortIdxReadVec).foreach { case ((issueResp, issueTimer), deqPortIdx) =>
      val Resp = resps(issueTimer)(deqPortIdx)
      issueResp := Resp
    }
  }

  //deq
  val enqEntryOldest = io.deq.map { deq =>
    Mux1H(deq.enqEntryOldestSel.bits, entries.take(EnqEntryNum))
  }
  val enqEntryOldestCancel = io.deq.map { deq =>
    Mux1H(deq.enqEntryOldestSel.bits, cancelByOg0Vec.getOrElse(VecInit(Seq.fill(params.numEntries)(false.B))).take(EnqEntryNum))
  }
  val othersEntryOldest = io.deq.map { deq =>
    Mux1H(deq.othersEntryOldestSel.bits, entries.drop(EnqEntryNum))
  }
  val othersEntryOldestCancel = io.deq.map { deq =>
    Mux1H(deq.othersEntryOldestSel.bits, cancelByOg0Vec.getOrElse(VecInit(Seq.fill(params.numEntries)(false.B))).drop(EnqEntryNum))
  }

  if (params.deqFuSame) {
    val subDeqPolicyEntryVec = Wire(Vec(params.numDeq, ValidIO(new EntryBundle)))
    val subDeqPolicyValidVec = Wire(Vec(params.numDeq, Bool()))
    val subDeqPolicyCancelByOg0Vec = Wire(Vec(params.numDeq, Bool()))

    subDeqPolicyEntryVec(0) := PriorityMux(io.deq(0).subDeqRequest.get, entries)
    subDeqPolicyEntryVec(1) := PriorityMux(Reverse(io.deq(0).subDeqRequest.get), entries.reverse)
    subDeqPolicyValidVec(0) := PopCount(io.deq(0).subDeqRequest.get) >= 1.U
    subDeqPolicyValidVec(1) := PopCount(io.deq(0).subDeqRequest.get) >= 2.U
    subDeqPolicyCancelByOg0Vec(0) := PriorityMux(io.deq(0).subDeqRequest.get, cancelByOg0Vec.getOrElse(VecInit(Seq.fill(params.numEntries)(false.B))))
    subDeqPolicyCancelByOg0Vec(1) := PriorityMux(Reverse(io.deq(0).subDeqRequest.get), cancelByOg0Vec.getOrElse(VecInit(Seq.fill(params.numEntries)(false.B))).reverse)

    io.deq(0).deqEntry := Mux(io.deq(0).othersEntryOldestSel.valid, othersEntryOldest(0), subDeqPolicyEntryVec(1))
    io.deq(1).deqEntry := subDeqPolicyEntryVec(0)
    io.cancelDeqVec(0) := Mux(io.deq(0).othersEntryOldestSel.valid, othersEntryOldestCancel(0), subDeqPolicyCancelByOg0Vec(1))
    io.cancelDeqVec(1) := subDeqPolicyCancelByOg0Vec(0)

    when (subDeqPolicyValidVec(0)) {
      assert(Mux1H(io.deq(0).subDeqSelOH.get, entries).bits.status.robIdx === subDeqPolicyEntryVec(0).bits.status.robIdx, "subDeqSelOH(0) is not the same\n")
    }
    when (subDeqPolicyValidVec(1)) {
      assert(Mux1H(io.deq(1).subDeqSelOH.get, entries).bits.status.robIdx === subDeqPolicyEntryVec(1).bits.status.robIdx, "subDeqSelOH(1) is not the same\n")
    }
  }
  else {
    io.deq.zipWithIndex.foreach { case (x, i) =>
      x.deqEntry := Mux(io.deq(i).othersEntryOldestSel.valid, othersEntryOldest(i), enqEntryOldest(i))
      io.cancelDeqVec(i) := Mux(io.deq(i).othersEntryOldestSel.valid, othersEntryOldestCancel(i), enqEntryOldestCancel(i))
    }
  }

  if (params.hasIQWakeUp) {
    cancelByOg0Vec.get.zip(srcWakeUpL1ExuOHVec.get).zip(srcTimerVec.get).foreach{ case ((cancelByOg0: Bool, l1ExuOH: Vec[UInt]), srcTimer: Vec[UInt]) =>
      cancelByOg0 := l1ExuOH.zip(srcTimer).map {
        case(exuOH, srcTimer) =>
          (exuOH.asUInt & io.og0Cancel.asUInt).orR && srcTimer === 1.U
      }.reduce(_ | _)
    }
  }

  io.valid := validVec.asUInt
  io.canIssue := canIssueVec.asUInt
  io.clear := clearVec.asUInt
  io.fuType := fuTypeVec
  io.dataSources := dataSourceVec
  io.srcWakeUpL1ExuOH.foreach(_ := srcWakeUpL1ExuOHVec.get)
  io.srcTimer.foreach(_ := srcTimerVec.get)
  io.cancel.foreach(_ := cancelVec.get)
  io.robIdx.foreach(_ := robIdxVec)
  io.uopIdx.foreach(_ := uopIdxVec.get)
  io.rsFeedback := 0.U.asTypeOf(io.rsFeedback) //todo
  io.deq.foreach{ x =>
    x.isFirstIssue := x.deqSelOH.valid && Mux1H(x.deqSelOH.bits, isFirstIssueVec)
  }
  if(backendParams.debugEn) {
    dontTouch(io.deq)
  }
  io.transSelVec.zip(transSelVec).foreach { case (sink, source) =>
    sink := source.asUInt
  }
}
