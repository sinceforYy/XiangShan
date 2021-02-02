package xiangshan.mem

import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xiangshan.cache._

class MaskedSyncDataModuleTemplate[T <: Data](gen: T, numEntries: Int, numRead: Int, numWrite: Int, numMRead: Int = 0, numMWrite: Int = 0) extends XSModule with HasDCacheParameters {
  val io = IO(new Bundle {
    // address indexed sync read
    val raddr = Input(Vec(numRead, UInt(log2Up(numEntries).W)))
    val rdata = Output(Vec(numRead, gen))
    // masked sync read (1H)
    val mrmask = Input(Vec(numMRead, Vec(numEntries, Bool())))
    val mrdata = Output(Vec(numMRead, gen))
    // address indexed write
    val wen   = Input(Vec(numWrite, Bool()))
    val waddr = Input(Vec(numWrite, UInt(log2Up(numEntries).W)))
    val wdata = Input(Vec(numWrite, gen))
    // masked write
    val mwmask = Input(Vec(numMWrite, Vec(numEntries, Bool())))
    val mwdata = Input(Vec(numMWrite, gen))
  })

  val data = Reg(Vec(numEntries, gen))

  // read ports
  for (i <- 0 until numRead) {
    io.rdata(i) := data(RegNext(io.raddr(i)))
  }

  // masked read ports
  for (i <- 0 until numMRead) {
    io.mrdata(i) := Mux1H(RegNext(io.mrmask(i)), data)
  }

  // write ports (with priorities)
  for (i <- 0 until numWrite) {
    when (io.wen(i)) {
      data(io.waddr(i)) := io.wdata(i)
    }
  }

  // masked write 
  for (j <- 0 until numEntries) {
    val wen = VecInit((0 until numMWrite).map(i => io.mwmask(i)(j))).asUInt.orR
    when (wen) {
      data(j) := VecInit((0 until numMWrite).map(i => {
        Mux(io.mwmask(i)(j), io.mwdata(i), 0.U).asUInt
      })).reduce(_ | _)
    }
  }

  // DataModuleTemplate should not be used when there're any write conflicts
  for (i <- 0 until numWrite) {
    for (j <- i+1 until numWrite) {
      assert(!(io.wen(i) && io.wen(j) && io.waddr(i) === io.waddr(j)))
    }
  }
}
