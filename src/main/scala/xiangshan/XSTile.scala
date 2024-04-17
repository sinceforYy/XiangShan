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

package xiangshan

import org.chipsalliance.cde.config.{Config, Parameters}
import chisel3._
import chisel3.util.{Valid, ValidIO}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.tile.{BusErrorUnit, BusErrorUnitParams, BusErrors}
import freechips.rocketchip.tilelink._
import coupledL2.{L2ParamKey, CoupledL2}
import system.HasSoCParameter
import top.BusPerfMonitor
import utility.{DelayN, ResetGen, TLClientsMerger, TLEdgeBuffer, TLLogger}
import xiangshan.backend.fu.NewCSR.{AIAToCSRBundle, CSRToAIABundle}

class XSTile()(implicit p: Parameters) extends LazyModule
  with HasXSParameter
  with HasSoCParameter
{
  override def shouldBeInlined: Boolean = false
  val core = LazyModule(new XSCore())
  val l2top = LazyModule(new L2Top())

  // =========== Public Ports ============
  val core_l3_pf_port = core.memBlock.l3_pf_sender_opt
  val memory_port = l2top.memory_port
  val uncache = l2top.mmio_port
  val beu_int_source = l2top.beu.intNode
  val core_reset_sink = BundleBridgeSink(Some(() => Reset()))
  val clint_int_node = l2top.clint_int_node
  val plic_int_node = l2top.plic_int_node
  val debug_int_node = l2top.debug_int_node
  core.memBlock.clint_int_sink := clint_int_node
  core.memBlock.plic_int_sink :*= plic_int_node
  core.memBlock.debug_int_sink := debug_int_node

  // =========== Components' Connection ============
  // L1 to l1_xbar
  coreParams.dcacheParametersOpt.map { _ =>
    l2top.misc_l2_pmu := l2top.l1d_logger := core.memBlock.dcache_port :=
      core.memBlock.l1d_to_l2_buffer.node := core.memBlock.dcache.clientNode
  }

  l2top.misc_l2_pmu := l2top.l1i_logger := core.memBlock.frontendBridge.icache_node
  if (!coreParams.softPTW) {
    l2top.misc_l2_pmu := l2top.ptw_logger := l2top.ptw_to_l2_buffer.node := core.memBlock.ptw_to_l2_buffer.node
  }
  l2top.l1_xbar :=* l2top.misc_l2_pmu

  val l2cache = l2top.l2cache
  // l1_xbar to l2
  l2cache match {
    case Some(l2) =>
      l2.node :*= l2top.xbar_l2_buffer :*= l2top.l1_xbar
      l2.pf_recv_node.map(recv => {
        println("Connecting L1 prefetcher to L2!")
        recv := core.memBlock.l2_pf_sender_opt.get
      })
    case None =>
  }

  val core_l3_tpmeta_source_port = l2cache match {
    case Some(l2) => l2.tpmeta_source_node
    case None => None
  }
  val core_l3_tpmeta_sink_port = l2cache match {
    case Some(l2) => l2.tpmeta_sink_node
    case None => None
  }

  // mmio
  l2top.i_mmio_port := l2top.i_mmio_buffer.node := core.memBlock.frontendBridge.instr_uncache_node
  l2top.d_mmio_port := core.memBlock.uncache.clientNode

  // =========== IO Connection ============
  class XSTileImp(wrapper: LazyModule) extends LazyModuleImp(wrapper) {
    val io = IO(new Bundle {
      val hartId = Input(UInt(64.W))
      val reset_vector = Input(UInt(PAddrBits.W))
      val cpu_halt = Output(Bool())
      val debugTopDown = new Bundle {
        val robHeadPaddr = Valid(UInt(PAddrBits.W))
        val l3MissMatch = Input(Bool())
      }
      val fromAIA = Flipped(Output(new AIAToCSRBundle))
      val toAIA = Output(new CSRToAIABundle)
    })

    dontTouch(io.hartId)

    val core_soft_rst = core_reset_sink.in.head._1 // unused

    l2top.module.hartId.fromTile := io.hartId
    core.module.io.hartId := l2top.module.hartId.toCore
    core.module.io.reset_vector := l2top.module.reset_vector.toCore
    l2top.module.reset_vector.fromTile := io.reset_vector
    l2top.module.cpu_halt.fromCore := core.module.io.cpu_halt
    io.cpu_halt := l2top.module.cpu_halt.toTile

    if (l2cache.isDefined) {
      // TODO: add perfEvents of L2
      // core.module.io.perfEvents.zip(l2cache.get.module.io.perfEvents.flatten).foreach(x => x._1.value := x._2)
      core.module.io.perfEvents <> DontCare
    }
    else {
      core.module.io.perfEvents <> DontCare
    }

    l2top.module.beu_errors.icache <> core.module.io.beu_errors.icache
    l2top.module.beu_errors.dcache <> core.module.io.beu_errors.dcache
    if (l2cache.isDefined) {
      // TODO: add ECC interface of L2
      l2top.module.beu_errors.l2 <> 0.U.asTypeOf(l2top.module.beu_errors.l2)
      core.module.io.l2_hint.bits.sourceId := l2top.module.l2_hint.bits
      core.module.io.l2_hint.valid := l2top.module.l2_hint.valid
      core.module.io.l2PfqBusy := false.B
      core.module.io.debugTopDown.l2MissMatch := l2top.module.debugTopDown.l2MissMatch
      l2top.module.debugTopDown.robHeadPaddr := core.module.io.debugTopDown.robHeadPaddr
    } else {
      l2top.module.beu_errors.l2 <> 0.U.asTypeOf(l2top.module.beu_errors.l2)
      core.module.io.l2_hint.bits.sourceId := l2top.module.l2_hint.bits
      core.module.io.l2_hint.valid := l2top.module.l2_hint.valid
      core.module.io.l2PfqBusy := false.B
      core.module.io.debugTopDown.l2MissMatch := false.B
    }

    io.debugTopDown.robHeadPaddr := core.module.io.debugTopDown.robHeadPaddr
    core.module.io.debugTopDown.l3MissMatch := io.debugTopDown.l3MissMatch

    // Modules are reset one by one
    // io_reset ----
    //             |
    //             v
    // reset ----> OR_SYNC --> {Misc, L2 Cache, Cores}
    // val resetChain = Seq(
    //   Seq(l2top.module, core.module)
    // )
    // ResetGen(resetChain, reset, !debugOpts.FPGAPlatform)

    // IMSIC
    core.module.io.fromAIA.rdata.valid        := io.fromAIA.rdata.valid
    core.module.io.fromAIA.rdata.bits.data    := io.fromAIA.rdata.bits.data
    core.module.io.fromAIA.rdata.bits.illegal := io.fromAIA.rdata.bits.illegal
    core.module.io.fromAIA.mtopei.valid       := io.fromAIA.mtopei.valid
    core.module.io.fromAIA.stopei.valid       := io.fromAIA.stopei.valid
    core.module.io.fromAIA.vstopei.valid      := io.fromAIA.vstopei.valid
    core.module.io.fromAIA.mtopei.bits        := io.fromAIA.mtopei.bits
    core.module.io.fromAIA.stopei.bits        := io.fromAIA.stopei.bits
    core.module.io.fromAIA.vstopei.bits       := io.fromAIA.vstopei.bits
    
    io.toAIA.addr.valid      := core.module.io.toAIA.addr.valid
    io.toAIA.addr.bits.addr  := core.module.io.toAIA.addr.bits.addr
    io.toAIA.addr.bits.prvm  := core.module.io.toAIA.addr.bits.prvm
    io.toAIA.addr.bits.v     := core.module.io.toAIA.addr.bits.v
    io.toAIA.vgein           := core.module.io.toAIA.vgein
    io.toAIA.mClaim          := core.module.io.toAIA.mClaim
    io.toAIA.sClaim          := core.module.io.toAIA.sClaim
    io.toAIA.vsClaim         := core.module.io.toAIA.vsClaim
    io.toAIA.wdata.valid     := core.module.io.toAIA.wdata.valid
    io.toAIA.wdata.bits.data := core.module.io.toAIA.wdata.bits.data
  }

  lazy val module = new XSTileImp(this)
}
