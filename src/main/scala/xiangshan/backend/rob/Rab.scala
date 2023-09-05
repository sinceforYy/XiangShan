package xiangshan.backend.rob

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import utility._
import xiangshan.backend.Bundles.DynInst
import xiangshan.backend.decode.VectorConstants
import xiangshan.backend.rename.SnapshotGenerator

class RenameBufferPtr(size: Int) extends CircularQueuePtr[RenameBufferPtr](size) {
  def this()(implicit p: Parameters) = this(p(XSCoreParamsKey).RabSize)
}

object RenameBufferPtr {
  def apply(flag: Boolean = false, v: Int = 0)(implicit p: Parameters): RenameBufferPtr = {
    val ptr = Wire(new RenameBufferPtr(p(XSCoreParamsKey).RabSize))
    ptr.flag := flag.B
    ptr.value := v.U
    ptr
  }
}

class RenameBufferEntry(implicit p: Parameters) extends RobCommitInfo {
  val robIdx = new RobPtr
}

class RenameBuffer(size: Int)(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper {
  val io = IO(new Bundle {
    val redirect = Input(ValidIO(new Bundle {
    }))

    val req = Vec(RenameWidth, Flipped(ValidIO(new DynInst)))

    val fromRob = new Bundle {
      val walkSize = Input(UInt(log2Up(size).W))
      val walkEnd = Input(Bool())
      val commitSize = Input(UInt(log2Up(size).W))
    }

    val snpt = Input(new SnapshotPort)

    val canEnq = Output(Bool())
    val enqPtrVec = Output(Vec(RenameWidth, new RenameBufferPtr))
    val vconfigPdest = Output(UInt(PhyRegIdxWidth.W))
    val commits = Output(new RobCommitIO)
    val diffCommits = Output(new DiffCommitIO)

    val status = Output(new Bundle {
      val walkEnd = Bool()
    })
  })

  // pointer
  private val enqPtrVec = RegInit(VecInit.tabulate(RenameWidth)(idx => RenameBufferPtr(flag = false, idx)))
  private val enqPtr = enqPtrVec.head
  private val enqPtrOH = RegInit(1.U(size.W))
  private val enqPtrOHShift = CircularShift(enqPtrOH)
  // may shift [0, RenameWidth] steps
  private val enqPtrOHVec = VecInit.tabulate(RenameWidth + 1)(enqPtrOHShift.left)
  private val enqPtrVecNext = Wire(enqPtrVec.cloneType)

  private val deqPtrVec = RegInit(VecInit.tabulate(CommitWidth)(idx => RenameBufferPtr(flag = false, idx)))
  private val deqPtr = deqPtrVec.head
  private val deqPtrOH = RegInit(1.U(size.W))
  private val deqPtrOHShift = CircularShift(deqPtrOH)
  private val deqPtrOHVec = VecInit.tabulate(CommitWidth + 1)(deqPtrOHShift.left)
  private val deqPtrVecNext = Wire(deqPtrVec.cloneType)
  XSError(deqPtr.toOH =/= deqPtrOH, p"wrong one-hot reg between $deqPtr and $deqPtrOH")

  private val walkPtr = Reg(new RenameBufferPtr)
  private val walkPtrOH = walkPtr.toOH
  private val walkPtrOHVec = VecInit.tabulate(CommitWidth + 1)(CircularShift(walkPtrOH).left)
  private val walkPtrNext = Wire(new RenameBufferPtr)

  private val snptEnq = io.canEnq && io.req.head.valid && io.req.head.bits.snapshot
  private val walkPtrSnapshots = SnapshotGenerator(enqPtr, snptEnq, io.snpt.snptDeq, io.redirect.valid)
  // may shift [0, CommitWidth] steps
  val headPtrOHVec2 = VecInit(Seq.tabulate(CommitWidth * MaxUopSize + 1)(_ % size).map(step => deqPtrOHShift.left(step)))

  val vcfgPtrOH = RegInit(1.U(size.W))
  val vcfgPtrOHShift = CircularShift(vcfgPtrOH)
  // may shift [0, 2) steps
  val vcfgPtrOHVec = VecInit.tabulate(2)(vcfgPtrOHShift.left)

  val diffPtrOH = RegInit(1.U(size.W))
  val diffPtrOHShift = CircularShift(diffPtrOH)
  // may shift [0, CommitWidth * MaxUopSize] steps
  val diffPtrOHVec = VecInit(Seq.tabulate(CommitWidth * MaxUopSize + 1)(_ % size).map(step => diffPtrOHShift.left(step)))

  // Regs
  val renameBuffer = RegInit(VecInit(Seq.fill(size){0.U.asTypeOf(new RenameBufferEntry)}))

  val s_idle :: s_special_walk :: s_walk :: Nil = Enum(3)
  val state = RegInit(s_idle)
  val stateNext = WireInit(state) // otherwise keep state value

  private val robWalkEndReg = RegInit(false.B)
  private val robWalkEnd = io.fromRob.walkEnd || robWalkEndReg

  when(io.redirect.valid) {
    robWalkEndReg := false.B
  }.elsewhen(io.fromRob.walkEnd) {
    robWalkEndReg := true.B
  }

  val realNeedAlloc = io.req.map(req => req.valid && req.bits.needWriteRf)
  val enqCount    = PopCount(realNeedAlloc)
  val commitCount = Mux(io.commits.isCommit && !io.commits.isWalk, PopCount(io.commits.commitValid), 0.U)
  val walkCount   = Mux(io.commits.isWalk && !io.commits.isCommit, PopCount(io.commits.walkValid), 0.U)
  val specialWalkCount = Mux(io.commits.isCommit && io.commits.isWalk, PopCount(io.commits.walkValid), 0.U)

  // number of pair(ldest, pdest) ready to commit to arch_rat
  val commitSize = RegInit(0.U(log2Up(size).W))
  val walkSize = RegInit(0.U(log2Up(size).W))
  val specialWalkSize = RegInit(0.U(log2Up(size).W))

  val newCommitSize = io.fromRob.commitSize
  val newWalkSize = io.fromRob.walkSize

  val commitSizeNxt = commitSize + newCommitSize - commitCount
  val walkSizeNxt = walkSize + newWalkSize - walkCount

  val newSpecialWalkSize = Mux(io.redirect.valid && !io.snpt.useSnpt, commitSizeNxt, 0.U)
  val specialWalkSizeNext = specialWalkSize + newSpecialWalkSize - specialWalkCount

  commitSize := Mux(io.redirect.valid && !io.snpt.useSnpt, 0.U, commitSizeNxt)
  specialWalkSize := specialWalkSizeNext
  walkSize := Mux(io.redirect.valid, 0.U, walkSizeNxt)

  walkPtrNext := MuxCase(walkPtr, Seq(
    (state === s_idle && stateNext === s_walk) -> walkPtrSnapshots(io.snpt.snptSelect),
    (state === s_special_walk && stateNext === s_walk) -> deqPtrVecNext.head,
    (state === s_walk) -> (walkPtr + walkCount),
  ))

  walkPtr := walkPtrNext

  val walkCandidates   = VecInit(walkPtrOHVec.map(sel => Mux1H(sel, renameBuffer)))
  val commitCandidates = VecInit(deqPtrOHVec.map(sel => Mux1H(sel, renameBuffer)))
  val vcfgCandidates   = VecInit(vcfgPtrOHVec.map(sel => Mux1H(sel, renameBuffer)))
  val diffCandidates   = VecInit(diffPtrOHVec.map(sel => Mux1H(sel, renameBuffer)))

  // update diff pointer
  val diffPtrOHNext = Mux(state === s_idle, diffPtrOHVec(newCommitSize), diffPtrOH)
  diffPtrOH := diffPtrOHNext

  // update vcfg pointer
  vcfgPtrOH := diffPtrOHNext

  // update enq pointer
  val enqPtrNext = Mux(
    state === s_walk && stateNext === s_idle,
    walkPtrNext,
    enqPtr + enqCount
  )
  val enqPtrOHNext = Mux(
    state === s_walk && stateNext === s_idle,
    walkPtrNext.toOH,
    enqPtrOHVec(enqCount)
  )
  enqPtr := enqPtrNext
  enqPtrOH := enqPtrOHNext
  enqPtrVecNext.zipWithIndex.map{ case(ptr, i) => ptr := enqPtrNext + i.U }
  enqPtrVec := enqPtrVecNext

  val deqPtrSteps = Mux1H(Seq(
    (state === s_idle) -> commitCount,
    (state === s_special_walk) -> specialWalkCount,
  ))

  // update deq pointer
  val deqPtrNext = deqPtr + deqPtrSteps
  val deqPtrOHNext = deqPtrOHVec(deqPtrSteps)
  deqPtr := deqPtrNext
  deqPtrOH := deqPtrOHNext
  deqPtrVecNext.zipWithIndex.map{ case(ptr, i) => ptr := deqPtrNext + i.U }
  deqPtrVec := deqPtrVecNext

  val allocatePtrVec = VecInit((0 until RenameWidth).map(i => enqPtrVec(PopCount(realNeedAlloc.take(i))).value))
  allocatePtrVec.zip(io.req).zip(realNeedAlloc).map{ case((allocatePtr, req), realNeedAlloc) =>
    when(realNeedAlloc){
      renameBuffer(allocatePtr).ldest := req.bits.ldest
      renameBuffer(allocatePtr).pdest := req.bits.pdest
      renameBuffer(allocatePtr).rfWen := req.bits.rfWen
      renameBuffer(allocatePtr).fpWen := req.bits.fpWen
      renameBuffer(allocatePtr).vecWen := req.bits.vecWen
      renameBuffer(allocatePtr).isMove := req.bits.eliminatedMove
      renameBuffer(allocatePtr).robIdx := req.bits.robIdx
    }
  }

  io.commits.isCommit := state === s_idle || state === s_special_walk
  io.commits.isWalk := state === s_walk || state === s_special_walk

  for(i <- 0 until CommitWidth) {
    io.commits.commitValid(i) := state === s_idle && i.U < commitSize || state === s_special_walk && i.U < specialWalkSize
    io.commits.walkValid(i) := state === s_walk && i.U < walkSize || state === s_special_walk && i.U < specialWalkSize
    // special walk use commitPtr
    io.commits.info(i) := Mux(state === s_idle || state === s_special_walk, commitCandidates(i), walkCandidates(i))
    // Todo: remove this
    io.commits.robIdx(i) := Mux(state === s_idle || state === s_special_walk, commitCandidates(i).robIdx, walkCandidates(i).robIdx)
  }

  private val walkEndNext = walkSizeNxt === 0.U
  private val specialWalkEndNext = specialWalkSizeNext === 0.U

  // change state
  state := stateNext
  when(io.redirect.valid) {
    when(io.snpt.useSnpt) {
      stateNext := s_walk
    }.otherwise {
      stateNext := s_special_walk
    }
  }.otherwise {
    // change stateNext
    switch(state) {
      // this transaction is not used actually, just list all states
      is(s_idle) {
        stateNext := s_idle
      }
      is(s_special_walk) {
        when(specialWalkEndNext) {
          stateNext := s_walk
        }
      }
      is(s_walk) {
        when(robWalkEnd && walkEndNext) {
          stateNext := s_idle
        }
      }
    }
  }

  val allowEnqueue = RegInit(true.B)
  val numValidEntries = distanceBetween(enqPtr, deqPtr)
  allowEnqueue := numValidEntries + enqCount <= (size - RenameWidth).U && state === s_idle
  io.canEnq := allowEnqueue
  io.enqPtrVec := enqPtrVec

  io.status.walkEnd := walkEndNext

  io.vconfigPdest := Mux(vcfgCandidates(0).ldest === VCONFIG_IDX.U && vcfgCandidates(0).vecWen, vcfgCandidates(0).pdest, vcfgCandidates(1).pdest)

  // for difftest
  io.diffCommits := 0.U.asTypeOf(new DiffCommitIO)
  io.diffCommits.isCommit := state === s_idle || state === s_special_walk
  for(i <- 0 until CommitWidth * MaxUopSize) {
    io.diffCommits.commitValid(i) := (state === s_idle || state === s_special_walk) && i.U < newCommitSize
    io.diffCommits.info(i) := diffCandidates(i)
  }

  XSError(isBefore(enqPtr, deqPtr) && !isFull(enqPtr, deqPtr), "\ndeqPtr is older than enqPtr!\n")

  QueuePerf(RabSize, numValidEntries, !allowEnqueue)
}
