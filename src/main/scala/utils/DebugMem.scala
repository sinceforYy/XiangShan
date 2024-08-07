/***************************************************************************************
* Copyright (c) 2024 Beijing Institute of Open Source Chip (BOSC)
* Copyright (c) 2020-2024 Institute of Computing Technology, Chinese Academy of Sciences
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

package utils

import chisel3._

object DebugMem {
  def apply[T <: Data](size: Int, data: T): DebugMem[T] = {
    new DebugMem(size, data)
  }
}

class DebugMem[T <: Data](size: Int, data: T) extends IndexedSeq[T] {
  private val debugReg: Vec[T] = Reg(Vec(size, data))

  def apply(addr: Int): T = debugReg(addr)

  def apply(addr: UInt): T = debugReg(addr)

  def length: Int = debugReg.length
}
