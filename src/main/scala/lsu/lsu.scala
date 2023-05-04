//******************************************************************************
// Copyright (c) 2012 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Out-of-Order Load/Store Unit
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Load/Store Unit is made up of the Load-Address Queue, the Store-Address
// Queue, and the Store-Data queue (LAQ, SAQ, and SDQ).
//
// Stores are sent to memory at (well, after) commit, loads are executed
// optimstically ASAP.  If a misspeculation was discovered, the pipeline is
// cleared. Loads put to sleep are retried.  If a LoadAddr and StoreAddr match,
// the Load can receive its data by forwarding data out of the Store-Data
// Queue.
//
// Currently, loads are sent to memory immediately, and in parallel do an
// associative search of the SAQ, on entering the LSU. If a hit on the SAQ
// search, the memory request is killed on the next cycle, and if the SDQ entry
// is valid, the store data is forwarded to the load (delayed to match the
// load-use delay to delay with the write-port structural hazard). If the store
// data is not present, or it's only a partial match (SB->LH), the load is put
// to sleep in the LAQ.
//
// Memory ordering violations are detected by stores at their addr-gen time by
// associatively searching the LAQ for newer loads that have been issued to
// memory.
//
// The store queue contains both speculated and committed stores.
//
// Only one port to memory... loads and stores have to fight for it, West Side
// Story style.
//
// TODO:
//    - Add predicting structure for ordering failures
//    - currently won't STD forward if DMEM is busy
//    - ability to turn off things if VM is disabled
//    - reconsider port count of the wakeup, retry stuff

package boom.lsu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.{Str, BooleanToAugmentedBoolean}

import boom.common._
import boom.exu.{BrUpdateInfo, Exception, FuncUnitResp, CommitSignals, ExeUnitResp}
import boom.util.{BoolToChar, AgePriorityEncoder, IsKilledByBranch, GetNewBrMask, WrapInc, IsOlder, UpdateBrMask}
import lsu.TCacheLSUIO
import lsu.TCacheRequestTypeEnum

class LSUExeIO(implicit p: Parameters) extends BoomBundle()(p)
{
  // The "resp" of the maddrcalc is really a "req" to the LSU
  val req       = Flipped(new ValidIO(new FuncUnitResp(xLen)))
  // Send load data to regfiles
  val iresp    = new DecoupledIO(new boom.exu.ExeUnitResp(xLen))
  val fresp    = new DecoupledIO(new boom.exu.ExeUnitResp(xLen+1)) // TODO: Should this be fLen?
}

class BoomDCacheReq(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomUOP
{
  val addr  = UInt(coreMaxAddrBits.W)
  val data  = Bits(coreDataBits.W)
  val is_hella = Bool() // Is this the hellacache req? If so this is not tracked in LDQ or STQ
  // val is_tcache = useMTE.option(Bool()) // Is this a request originating from the tag cache?
  // val byte_wmask = useMTE.option(Bits(8.W)) // MTE requires sub-byte writes to dcache for tags
}

class BoomDCacheResp(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomUOP
{
  val data = Bits(coreDataBits.W)
  val is_hella = Bool()
  // val is_tcache = useMTE.option(Bool()) // Is this a request originating from the tag cache?
}

class LSUDMemIO(implicit p: Parameters, edge: TLEdgeOut) extends BoomBundle()(p)
{
  // In LSU's dmem stage, send the request
  val req         = new DecoupledIO(Vec(memWidth, Valid(new BoomDCacheReq)))
  // In LSU's LCAM search stage, kill if order fail (or forwarding possible)
  val s1_kill     = Output(Vec(memWidth, Bool()))
  // Get a request any cycle
  val resp        = Flipped(Vec(memWidth, new ValidIO(new BoomDCacheResp)))
  // In our response stage, if we get a nack, we need to reexecute
  val nack        = Flipped(Vec(memWidth, new ValidIO(new BoomDCacheReq)))

  val brupdate       = Output(new BrUpdateInfo)
  val exception    = Output(Bool())
  val rob_pnr_idx  = Output(UInt(robAddrSz.W))
  val rob_head_idx = Output(UInt(robAddrSz.W))

  val release = Flipped(new DecoupledIO(new TLBundleC(edge.bundle)))

  // Clears prefetching MSHRs
  val force_order  = Output(Bool())
  val ordered     = Input(Bool())

  val perf = Input(new Bundle {
    val acquire = Bool()
    val release = Bool()
  })

}

class MTEFaultPacket(implicit p: Parameters) extends BoomBundle()(p) 
  with HasBoomUOP {
  val faulting_address = UInt(coreMaxAddrBits.W)
  val is_load = Bool()
  val mem_size = UInt(2.W)
  val physical_tag = UInt(MTEConfig.tagBits.W)
  val address_tag = UInt(MTEConfig.tagBits.W)
}

class LSUCoreIO(implicit p: Parameters) extends BoomBundle()(p)
{
  val exe = Vec(memWidth, new LSUExeIO)

  val dis_uops    = Flipped(Vec(coreWidth, Valid(new MicroOp)))
  val dis_ldq_idx = Output(Vec(coreWidth, UInt(ldqAddrSz.W)))
  val dis_stq_idx = Output(Vec(coreWidth, UInt(stqAddrSz.W)))

  val ldq_full    = Output(Vec(coreWidth, Bool()))
  val stq_full    = Output(Vec(coreWidth, Bool()))

  val fp_stdata   = Flipped(Decoupled(new ExeUnitResp(fLen)))

  val commit      = Input(new CommitSignals)
  val commit_load_at_rob_head = Input(Bool())

  // Stores clear busy bit when stdata is received
  // memWidth for int, 1 for fp (to avoid back-pressure fpstdat)
  val clr_bsy         = Output(Vec(memWidth + 1, Valid(UInt(robAddrSz.W))))

  // Speculatively safe load (barring memory ordering failure)
  val clr_unsafe      = Output(Vec(memWidth, Valid(UInt(robAddrSz.W))))

  // Tell the DCache to clear prefetches/speculating misses
  val fence_dmem   = Input(Bool())

  // Speculatively tell the IQs that we'll get load data back next cycle
  val spec_ld_wakeup = Output(Vec(memWidth, Valid(UInt(maxPregSz.W))))
  // Tell the IQs that the load we speculated last cycle was misspeculated
  val ld_miss      = Output(Bool())

  val brupdate       = Input(new BrUpdateInfo)
  val rob_pnr_idx  = Input(UInt(robAddrSz.W))
  val rob_head_idx = Input(UInt(robAddrSz.W))
  val exception    = Input(Bool())

  val fencei_rdy  = Output(Bool())

  val lxcpt       = Output(Valid(new Exception))

  val tsc_reg     = Input(UInt())

  val perf        = Output(new Bundle {
    val acquire = Bool()
    val release = Bool()
    val tlbMiss = Bool()
  })

  val mte_enabled = useMTE.option(Input(Bool()))
  val mte_permissive_tag = useMTE.option(Input(UInt(MTEConfig.tagBits.W)))
  val mte_fault_packet = useMTE.option(Output(Valid(new MTEFaultPacket)))
}

class LSUIO(implicit p: Parameters, edge: TLEdgeOut) extends BoomBundle()(p)
{
  val ptw   = new rocket.TLBPTWIO
  val core  = new LSUCoreIO
  val dmem  = new LSUDMemIO
  val tcache:Option[TCacheLSUIO] = {
    if (useMTE) {
      Some(Flipped(new TCacheLSUIO()(p=p, tcacheParams=boomTileParams.tcache.get)))
    } else {
      None
    }
  }

  val hellacache = Flipped(new freechips.rocketchip.rocket.HellaCacheIO)
}

class LDQEntry(implicit p: Parameters) extends BoomBundle()(p)
    with HasBoomUOP
{
  val addr                = Valid(UInt(coreMaxAddrBits.W))
  val addr_is_virtual     = Bool() // Virtual address, we got a TLB miss
  val addr_is_uncacheable = Bool() // Uncacheable, wait until head of ROB to execute
  val addr_mte_tag        = useMTE.option(UInt(MTEConfig.tagBits.W)) // tag from address
  val phys_mte_tag        = useMTE.option(Valid(UInt(MTEConfig.tagBits.W))) // the memory's actual tag

  val executed            = Bool() // load sent to memory, reset by NACKs
  /**
   * Have the tag check requests been launched into the tag cache?
   * Note that this does not imply that tags have yet arrived, see the valid bit
   * on the phys_mte_tag.
   */
  val phys_mte_tag_pending = useMTE.option(Bool()) //tag fetch sent to memory but has not returned. Cleared on response.
  /**
    * Data has been received from dmem and transmitted back to the core
    * (i.e. this load has been released)
    */
  val succeeded           = Bool()
  /**
   * The core has committed this instruction. This doesn't necessarily mean
   * we're done with the entry as we still need to wait for both data and tag
   * check
   */
  val committed           = Bool()
  val order_fail          = Bool()
  val observed            = Bool()

  val st_dep_mask         = UInt(numStqEntries.W) // list of stores older than us
  val youngest_stq_idx    = UInt(stqAddrSz.W) // index of the oldest store younger than us

  val forward_std_val     = Bool()
  val forward_stq_idx     = UInt(stqAddrSz.W) // Which store did we get the store-load forward from?

  val debug_wb_data       = UInt(xLen.W)
}

class STQEntry(implicit p: Parameters) extends BoomBundle()(p)
   with HasBoomUOP
{
  val addr                = Valid(UInt(coreMaxAddrBits.W))
  val addr_is_virtual     = Bool() // Virtual address, we got a TLB miss
  val addr_mte_tag        = useMTE.option(UInt(MTEConfig.tagBits.W)) // tag from address
  val phys_mte_tag        = useMTE.option(Valid(UInt(MTEConfig.tagBits.W))) // the memory's actual tag
  val data                = Valid(UInt(xLen.W))

  val committed           = Bool() // committed by ROB
  val phys_mte_tag_pending = useMTE.option(Bool()) //tag fetch sent to memory but has not returned. Cleared on response.
  val commit_in_flight    = Bool()

  val succeeded           = Bool() // D$ has ack'd this, we don't need to maintain this anymore

  val debug_wb_data       = UInt(xLen.W)
}

class LSU(implicit p: Parameters, edge: TLEdgeOut) extends BoomModule()(p)
  with rocket.HasL1HellaCacheParameters
{
  val io = IO(new LSUIO)


  val ldq = Reg(Vec(numLdqEntries, Valid(new LDQEntry)))
  val stq = Reg(Vec(numStqEntries, Valid(new STQEntry)))



  val ldq_head         = Reg(UInt(ldqAddrSz.W))
  val ldq_tail         = Reg(UInt(ldqAddrSz.W))
  val stq_head         = Reg(UInt(stqAddrSz.W)) // point to next store to clear from STQ (i.e., send to memory)
  val stq_tail         = Reg(UInt(stqAddrSz.W))
  val stq_commit_head  = Reg(UInt(stqAddrSz.W)) // point to next store to commit
  val stq_execute_head = Reg(UInt(stqAddrSz.W)) // point to next store to execute
  val ldq_head_e = ldq(ldq_head)
  val ldq_head_e_mte_phys_completed = {
    if (!useMTE) {
      true.B
    } else {
      ldq_head_e.bits.phys_mte_tag.get.valid
    }
  }
val ldq_head_e_mte_phys_mte_tag_pending = ldq_head_e.bits.phys_mte_tag_pending.getOrElse(false.B)
  /*
  0 for tcache
  1 for dmem
  */
  val stq_execute_head_rewinds = Wire(Vec(2, Valid(UInt(stqAddrSz.W))))
  for (i <- 0 until stq_execute_head_rewinds.length) {
    stq_execute_head_rewinds(i).valid := false.B
    stq_execute_head_rewinds(i).bits := DontCare
  }


  // def update_stq_execute_head_rewind(rewind:UInt) = {
  //   when (!stq_execute_head_rewind_valid) {
  //     stq_execute_head_rewind_valid = true.B
  //     stq_execute_head_rewind_idx = rewind
  //   } .elsewhen (IsOlder(rewind, stq_execute_head_rewind_idx, stq_head)) {
  //     stq_execute_head_rewind_idx = rewind
  //   }
  // }


  // If we got a mispredict, the tail will be misaligned for 1 extra cycle
  assert (io.core.brupdate.b2.mispredict ||
          stq(stq_execute_head).valid ||
          stq_head === stq_execute_head ||
          stq_tail === stq_execute_head,
            "stq_execute_head got off track.")

  val h_ready :: h_s1 :: h_s2 :: h_s2_nack :: h_wait :: h_replay :: h_dead :: Nil = Enum(7)
  // s1 : do TLB, if success and not killed, fire request go to h_s2
  //      store s1_data to register
  //      if tlb miss, go to s2_nack
  //      if don't get TLB, go to s2_nack
  //      store tlb xcpt
  // s2 : If kill, go to dead
  //      If tlb xcpt, send tlb xcpt, go to dead
  // s2_nack : send nack, go to dead
  // wait : wait for response, if nack, go to replay
  // replay : refire request, use already translated address
  // dead : wait for response, ignore it
  val hella_state           = RegInit(h_ready)
  val hella_req             = Reg(new rocket.HellaCacheReq)
  val hella_data            = Reg(new rocket.HellaCacheWriteData)
  val hella_paddr           = Reg(UInt(paddrBits.W))
  val hella_xcpt            = Reg(new rocket.HellaCacheExceptions)


  val dtlb = Module(new NBDTLB(
    instruction = false, lgMaxSize = log2Ceil(coreDataBytes), rocket.TLBConfig(dcacheParams.nTLBSets, dcacheParams.nTLBWays)))

  io.ptw <> dtlb.io.ptw
  io.core.perf.tlbMiss := io.ptw.req.fire
  io.core.perf.acquire := io.dmem.perf.acquire
  io.core.perf.release := io.dmem.perf.release



  val clear_store     = WireInit(false.B)
  val live_store_mask = RegInit(0.U(numStqEntries.W))
  var next_live_store_mask = Mux(clear_store, live_store_mask & ~(1.U << stq_head),
                                              live_store_mask)


  def widthMap[T <: Data](f: Int => T) = VecInit((0 until memWidth).map(f))


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Enqueue new entries
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // This is a newer store than existing loads, so clear the bit in all the store dependency masks
  for (i <- 0 until numLdqEntries)
  {
    when (clear_store)
    {
      ldq(i).bits.st_dep_mask := ldq(i).bits.st_dep_mask & ~(1.U << stq_head)
    }
  }

  // Decode stage
  var ld_enq_idx = ldq_tail
  var st_enq_idx = stq_tail

  val stq_nonempty = (0 until numStqEntries).map{ i => stq(i).valid }.reduce(_||_) =/= 0.U

  var ldq_full = Bool()
  var stq_full = Bool()

  for (w <- 0 until coreWidth)
  {
    ldq_full = WrapInc(ld_enq_idx, numLdqEntries) === ldq_head
    io.core.ldq_full(w)    := ldq_full
    io.core.dis_ldq_idx(w) := ld_enq_idx

    stq_full = WrapInc(st_enq_idx, numStqEntries) === stq_head
    io.core.stq_full(w)    := stq_full
    io.core.dis_stq_idx(w) := st_enq_idx

    val dis_ld_val = io.core.dis_uops(w).valid && io.core.dis_uops(w).bits.uses_ldq && !io.core.dis_uops(w).bits.exception
    val dis_st_val = io.core.dis_uops(w).valid && io.core.dis_uops(w).bits.uses_stq && !io.core.dis_uops(w).bits.exception
    when (dis_ld_val)
    {
      ldq(ld_enq_idx).valid                := true.B
      ldq(ld_enq_idx).bits.uop             := io.core.dis_uops(w).bits
      ldq(ld_enq_idx).bits.youngest_stq_idx  := st_enq_idx
      ldq(ld_enq_idx).bits.st_dep_mask     := next_live_store_mask

      ldq(ld_enq_idx).bits.addr.valid      := false.B
      ldq(ld_enq_idx).bits.executed        := false.B
      ldq(ld_enq_idx).bits.succeeded       := false.B
      ldq(ld_enq_idx).bits.order_fail      := false.B
      ldq(ld_enq_idx).bits.observed        := false.B
      ldq(ld_enq_idx).bits.forward_std_val := false.B

      assert (ld_enq_idx === io.core.dis_uops(w).bits.ldq_idx, "[lsu] mismatch enq load tag.")
      assert (!ldq(ld_enq_idx).valid, "[lsu] Enqueuing uop is overwriting ldq entries")
    }
      .elsewhen (dis_st_val)
    {
      stq(st_enq_idx).valid           := true.B
      stq(st_enq_idx).bits.uop        := io.core.dis_uops(w).bits
      stq(st_enq_idx).bits.addr.valid := false.B
      stq(st_enq_idx).bits.data.valid := false.B
      stq(st_enq_idx).bits.committed  := false.B
      stq(st_enq_idx).bits.succeeded  := false.B

      assert (st_enq_idx === io.core.dis_uops(w).bits.stq_idx, "[lsu] mismatch enq store tag.")
      assert (!stq(st_enq_idx).valid, "[lsu] Enqueuing uop is overwriting stq entries")
    }

    ld_enq_idx = Mux(dis_ld_val, WrapInc(ld_enq_idx, numLdqEntries),
                                 ld_enq_idx)

    next_live_store_mask = Mux(dis_st_val, next_live_store_mask | (1.U << st_enq_idx),
                                           next_live_store_mask)
    st_enq_idx = Mux(dis_st_val, WrapInc(st_enq_idx, numStqEntries),
                                 st_enq_idx)

    assert(!(dis_ld_val && dis_st_val), "A UOP is trying to go into both the LDQ and the STQ")
  }

  ldq_tail := ld_enq_idx
  stq_tail := st_enq_idx

  io.dmem.force_order   := io.core.fence_dmem
  io.core.fencei_rdy    := !stq_nonempty && io.dmem.ordered


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Execute stage (access TLB, send requests to Memory)
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // We can only report 1 exception per cycle.
  // Just be sure to report the youngest one
  val mem_xcpt_valid  = Wire(Bool())
  val mem_xcpt_cause  = Wire(UInt())
  val mem_xcpt_uop    = Wire(new MicroOp)
  val mem_xcpt_vaddr  = Wire(UInt())


  //---------------------------------------
  // Can-fire logic and wakeup/retry select
  //
  // First we determine what operations are waiting to execute.
  // These are the "can_fire"/"will_fire" signals

  val will_fire_load_incoming      = Wire(Vec(memWidth, Bool()))
  val will_fire_stad_incoming      = Wire(Vec(memWidth, Bool()))
  val will_fire_sta_incoming       = Wire(Vec(memWidth, Bool()))
  val will_fire_std_incoming       = Wire(Vec(memWidth, Bool()))
  val will_fire_sfence             = Wire(Vec(memWidth, Bool()))
  val will_fire_hella_incoming     = Wire(Vec(memWidth, Bool()))
  val will_fire_hella_wakeup       = Wire(Vec(memWidth, Bool()))
  val will_fire_release            = Wire(Vec(memWidth, Bool()))
  val will_fire_load_retry         = Wire(Vec(memWidth, Bool()))
  val will_fire_sta_retry          = Wire(Vec(memWidth, Bool()))
  val will_fire_store_commit       = Wire(Vec(memWidth, Bool()))
  val will_fire_load_wakeup        = Wire(Vec(memWidth, Bool()))
  val will_fire_ldq_tag_retry      = Wire(Vec(memWidth, Bool()))
  val will_fire_stq_tag_retry      = Wire(Vec(memWidth, Bool()))
  val will_fire_stta_incoming      = Wire(Vec(memWidth, Bool()))
  val will_fire_stta_retry         = Wire(Vec(memWidth, Bool()))
  val will_fire_stt_commit         = Wire(Vec(memWidth, Bool()))

  val exe_req = WireInit(VecInit(io.core.exe.map(_.req)))
  // Sfence goes through all pipes
  for (i <- 0 until memWidth) {
    when (io.core.exe(i).req.bits.sfence.valid) {
      exe_req := VecInit(Seq.fill(memWidth) { io.core.exe(i).req })
    }
  }

  // -------------------------------
  // Assorted signals for scheduling

  // Don't wakeup a load if we just sent it last cycle or two cycles ago
  // The block_load_mask may be wrong, but the executing_load mask must be accurate
  val block_load_mask    = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B)))
  val p1_block_load_mask = RegNext(block_load_mask)
  val p2_block_load_mask = RegNext(p1_block_load_mask)

 // Prioritize emptying the store queue when it is almost full
  val stq_almost_full = RegNext(WrapInc(WrapInc(st_enq_idx, numStqEntries), numStqEntries) === stq_head ||
                                WrapInc(st_enq_idx, numStqEntries) === stq_head)

  // The store at the commit head needs the DCache to appear ordered
  // Delay firing load wakeups and retries now
  val store_needs_order = WireInit(false.B)

  val ldq_incoming_idx = widthMap(i => exe_req(i).bits.uop.ldq_idx)
  val ldq_incoming_e   = widthMap(i => ldq(ldq_incoming_idx(i)))

  val stq_incoming_idx = widthMap(i => exe_req(i).bits.uop.stq_idx)
  val stq_incoming_e   = widthMap(i => stq(stq_incoming_idx(i)))

  val ldq_retry_idx = RegNext(AgePriorityEncoder((0 until numLdqEntries).map(i => {
    val e = ldq(i).bits
    val block = block_load_mask(i) || p1_block_load_mask(i)
    e.addr.valid && e.addr_is_virtual && !block
  }), ldq_head))
  val ldq_retry_e            = ldq(ldq_retry_idx)

  val stq_retry_idx = RegNext(AgePriorityEncoder((0 until numStqEntries).map(i => {
    val e = stq(i).bits
    e.addr.valid && e.addr_is_virtual
  }), stq_commit_head))
  val stq_retry_e   = stq(stq_retry_idx)

  val stq_commit_e  = stq(stq_execute_head)

  val ldq_wakeup_idx = RegNext(AgePriorityEncoder((0 until numLdqEntries).map(i=> {
    val e = ldq(i).bits
    val block = block_load_mask(i) || p1_block_load_mask(i)
    e.addr.valid && !e.executed && !e.succeeded && !e.addr_is_virtual && !block
  }), ldq_head))
  val ldq_wakeup_e   = ldq(ldq_wakeup_idx)

  def ldq_e_can_tag_retry(e:LDQEntry) = {
    if (!useMTE) {
      false.B
    } else {
      /*
      Allow launch of valid entries if they've completed TLB and don't have a tag already.
      While there is a possibility that an entry may be ineligible for ldq_retry but could
      fetch tags if we allowed virtual here and did TLB, this should happen fairly rarely
      since this only happens if we only have blocked loads in the queue.
      TODO: Track this as a metric--is this actually true?
      */
      e.addr.valid && !e.addr_is_virtual && 
        !e.phys_mte_tag_pending.get && !e.phys_mte_tag.get.valid
    }
  }

  // If the tag cache is blocked due to too many outstanding misses, tag operations
  // will be retried as soon as available
  val ldq_tag_retry_idx = RegNext(AgePriorityEncoder((0.until(numLdqEntries)).map(i => {
    val e = ldq(i).bits
    ldq(i).valid && ldq_e_can_tag_retry(e)
  }), ldq_head))
  val ldq_tag_retry_e = ldq(ldq_tag_retry_idx)

  def stq_e_can_tag_retry(e:STQEntry) = {
    if (!useMTE) {
      false.B
    } else {
      /*
      Allow launch of valid entries if they've completed TLB and don't have a tag already.
      */
      e.addr.valid && !e.addr_is_virtual && 
        !e.phys_mte_tag_pending.get && !e.phys_mte_tag.get.valid &&
        /* Do not fetch tags for tag write operations */
        !e.uop.is_mte_tag_write
    }
  }

  // If the tag cache is blocked due to too many outstanding misses, tag operations
  // will be retried as soon as available
  val stq_tag_retry_idx = RegNext(AgePriorityEncoder((0.until(numStqEntries)).map(i => {
    val e = stq(i).bits
    stq(i).valid && stq_e_can_tag_retry(e)
  }), stq_head))
  val stq_tag_retry_e = stq(stq_tag_retry_idx)


  def stq_e_can_stta_retry(e:STQEntry) = {
    if (!useMTE) {
      false.B
    } else {
      /*
      If the op hasn't already been translated, it's eligible for retry
      */
      e.addr.valid && e.addr_is_virtual && e.uop.is_mte_tag_write
    }
  }

  val stta_retry_idx = RegNext(AgePriorityEncoder((0.until(numStqEntries)).map(i => {
    val e = stq(i).bits
    stq(i).valid && stq_e_can_stta_retry(e)
  }), stq_head))
  val stta_retry_e = stq(stta_retry_idx)


  // -----------------------
  // Determine what can fire

  // Can we fire a incoming load
  val can_fire_load_incoming = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_load)

  // Can we fire an incoming store addrgen + store datagen
  val can_fire_stad_incoming = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_sta
                                                              && exe_req(w).bits.uop.ctrl.is_std)

  // Can we fire an incoming store addrgen
  val can_fire_sta_incoming  = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_sta
                                                              && !exe_req(w).bits.uop.ctrl.is_std)

  // Can we fire an incoming store datagen
  val can_fire_std_incoming  = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_std
                                                              && !exe_req(w).bits.uop.ctrl.is_sta)

  // Can we fire an incoming sfence
  val can_fire_sfence        = widthMap(w => exe_req(w).valid && exe_req(w).bits.sfence.valid)

  // Can we fire a request from dcache to release a line
  // This needs to go through LDQ search to mark loads as dangerous
  val can_fire_release       = widthMap(w => (w == memWidth-1).B && io.dmem.release.valid)
  io.dmem.release.ready     := will_fire_release.reduce(_||_)

  // Can we retry a load that missed in the TLB
  val can_fire_load_retry    = widthMap(w =>
                               ( ldq_retry_e.valid                            && 
                                 ldq_retry_e.bits.addr.valid                  &&
                                 ldq_retry_e.bits.addr_is_virtual             &&
                                !p1_block_load_mask(ldq_retry_idx)            &&
                                !p2_block_load_mask(ldq_retry_idx)            &&
                                RegNext(dtlb.io.miss_rdy)                     &&
                                !store_needs_order                            &&
                                (w == memWidth-1).B                           && // TODO: Is this best scheduling?
                                !ldq_retry_e.bits.order_fail))

  // Can we retry a store addrgen that missed in the TLB
  // - Weird edge case when sta_retry and std_incoming for same entry in same cycle. Delay this
  val can_fire_sta_retry     = widthMap(w =>
                               ( stq_retry_e.valid                            &&
                                 stq_retry_e.bits.addr.valid                  &&
                                 stq_retry_e.bits.addr_is_virtual             &&
                                 /* tag retries handled by can_fire_stta_retry */
                                 !stq_retry_e.bits.uop.is_mte_tag_write       &&
                                 (w == memWidth-1).B                          &&
                                 RegNext(dtlb.io.miss_rdy)                    &&
                                 !(widthMap(i => (i != w).B               &&
                                                 can_fire_std_incoming(i) &&
                                                 stq_incoming_idx(i) === stq_retry_idx).reduce(_||_))
                               ))
  // Can we commit a store
  val can_fire_store_commit  = widthMap(w =>
                               ( stq_commit_e.valid                           &&
                                !stq_commit_e.bits.uop.is_fence               &&
                                !stq_commit_e.bits.uop.is_mte_tag_write       &&
                                !mem_xcpt_valid                               &&
                                !stq_commit_e.bits.uop.exception              &&
                                (w == 0).B                                    &&
                                (stq_commit_e.bits.committed || ( stq_commit_e.bits.uop.is_amo      &&
                                                                  stq_commit_e.bits.addr.valid      &&
                                                                 !stq_commit_e.bits.addr_is_virtual &&
                                                                  stq_commit_e.bits.data.valid))))

  // Can we wakeup a load that was nack'd
  val block_load_wakeup = WireInit(false.B)
  val can_fire_load_wakeup = widthMap(w =>
                             ( ldq_wakeup_e.valid                                      &&
                               ldq_wakeup_e.bits.addr.valid                            &&
                              !ldq_wakeup_e.bits.succeeded                             &&
                              !ldq_wakeup_e.bits.addr_is_virtual                       &&
                              !ldq_wakeup_e.bits.executed                              &&
                              !ldq_wakeup_e.bits.order_fail                            &&
                              !p1_block_load_mask(ldq_wakeup_idx)                      &&
                              !p2_block_load_mask(ldq_wakeup_idx)                      &&
                              !store_needs_order                                       &&
                              !block_load_wakeup                                       &&
                              (w == memWidth-1).B                                      &&
                              (!ldq_wakeup_e.bits.addr_is_uncacheable || (io.core.commit_load_at_rob_head &&
                                                                          ldq_head === ldq_wakeup_idx &&
                                                                          ldq_wakeup_e.bits.st_dep_mask.asUInt === 0.U))))

  // Can we fire an incoming hellacache request
  val can_fire_hella_incoming  = WireInit(widthMap(w => false.B)) // This is assigned to in the hellashim ocntroller

  // Can we fire a hellacache request that the dcache nack'd
  val can_fire_hella_wakeup    = WireInit(widthMap(w => false.B)) // This is assigned to in the hellashim controller
  val can_fire_ldq_tag_retry = widthMap(w =>
     //XXX: Won't this tell ever lane they can fire this? Not an issue for our 1-wide requirement but it's unclear
     //who is responsible for making sure an op fires down only one lane at a time
     ldq_tag_retry_e.valid && ldq_e_can_tag_retry(ldq_tag_retry_e.bits)
  )
  val can_fire_stq_tag_retry = widthMap(w =>
    //XXX: ibd
    stq_tag_retry_e.valid && stq_e_can_tag_retry(stq_tag_retry_e.bits)
  )

  /** Can fire STore Tag Address -- translate the incoming address for a tag write */
  val can_fire_stta_incoming = widthMap(w =>
    exe_req(w).valid && exe_req(w).bits.uop.is_mte_tag_write
  )

  val can_fire_stta_retry = widthMap(w => 
    stta_retry_e.valid && stq_e_can_stta_retry(stta_retry_e.bits)
  )

  val can_fire_stt_commit    = widthMap(w =>
                              ( stq_commit_e.valid                          &&
                              stq_commit_e.bits.uop.is_mte_tag_write        &&
                              !mem_xcpt_valid                               &&
                              !stq_commit_e.bits.uop.exception              &&
                              (w == 0).B                                    &&
                              stq_commit_e.bits.committed                   &&
                              stq_commit_e.bits.addr.valid                  &&
                              !stq_commit_e.bits.addr_is_virtual            &&
                              stq_commit_e.bits.data.valid))
// val can_fire_load_clear_tag_lcam = widthMap(w =>
//   ldq_head_e.valid
  
// )

  //---------------------------------------------------------
  // Controller logic. Arbitrate which request actually fires

  val exe_tlb_valid = Wire(Vec(memWidth, Bool()))
  for (w <- 0 until memWidth) {
    var tlb_avail  = true.B
    var dc_avail   = true.B
    var lcam_avail = true.B
    var rob_avail  = true.B
    var tcache_avil= true.B

    def lsu_sched(can_fire: Bool, uses_tlb:Boolean, uses_dc:Boolean, uses_lcam: Boolean, uses_rob:Boolean, uses_tcache:Boolean): Bool = {
      val will_fire = can_fire && !(uses_tlb.B && !tlb_avail) &&
                                  !(uses_lcam.B && !lcam_avail) &&
                                  !(uses_dc.B && !dc_avail) &&
                                  !(uses_rob.B && !rob_avail) &&
                                  !((useMTE.B && uses_tcache.B && !tcache_avil))
      tlb_avail  = tlb_avail  && !(will_fire && uses_tlb.B)
      lcam_avail = lcam_avail && !(will_fire && uses_lcam.B)
      dc_avail   = dc_avail   && !(will_fire && uses_dc.B)
      rob_avail  = rob_avail  && !(will_fire && uses_rob.B)
      tcache_avil= tcache_avil && !(will_fire && uses_tcache.B)
      dontTouch(will_fire) // dontTouch these so we can inspect the will_fire signals
      will_fire
    }

    // The order of these statements is the priority
    // Some restrictions
    //  - Incoming ops must get precedence, can't backpresure memaddrgen
    //  - Incoming hellacache ops must get precedence over retrying ops (PTW must get precedence over retrying translation)
    // Notes on performance
    //  - Prioritize releases, this speeds up cache line writebacks and refills
    //  - Store commits are lowest priority, since they don't "block" younger instructions unless stq fills up
    //  - Tag retries are low priority as they don't block unless the queue/ROB is full
    will_fire_load_incoming (w) := lsu_sched(can_fire_load_incoming (w) , true , true , true , false, true)  // TLB , DC , LCAM ,     , TCACHE
    will_fire_stad_incoming (w) := lsu_sched(can_fire_stad_incoming (w) , true , false, true , true,  true)  // TLB ,    , LCAM , ROB , TCACHE
    will_fire_sta_incoming  (w) := lsu_sched(can_fire_sta_incoming  (w) , true , false, true , true,  true)  // TLB ,    , LCAM , ROB , TCACHE
    will_fire_std_incoming  (w) := lsu_sched(can_fire_std_incoming  (w) , false, false, false, true,  false) //                 , ROB
    if (useMTE) {
    //Even though STTA isn't really that high priority, we need to handling all incoming requests over retries
    //to avoid dropping it on the floor
    will_fire_stta_incoming (w) := lsu_sched(can_fire_stta_incoming (w) , true, false, false, true, false)  // TLB ,    ,      ,  ROB ,  
    } else {
      will_fire_stta_incoming (w) := false.B
    }
    will_fire_sfence        (w) := lsu_sched(can_fire_sfence        (w) , true , false, false, true,  false) // TLB ,    ,      , ROB
    will_fire_release       (w) := lsu_sched(can_fire_release       (w) , false, false, true , false, false) //            LCAM
    will_fire_hella_incoming(w) := lsu_sched(can_fire_hella_incoming(w) , true , true , false, false, false) // TLB , DC
    will_fire_hella_wakeup  (w) := lsu_sched(can_fire_hella_wakeup  (w) , false, true , false, false, false) //     , DC
    will_fire_load_retry    (w) := lsu_sched(can_fire_load_retry    (w) , true , true , true , false, true)  // TLB , DC , LCAM  ,    , TCACHE
    will_fire_sta_retry     (w) := lsu_sched(can_fire_sta_retry     (w) , true , false, true , true,  true)  // TLB ,    , LCAM , ROB , TCACHE// TODO: This should be higher priority
    if (useMTE) {
    will_fire_ldq_tag_retry (w) := lsu_sched(can_fire_ldq_tag_retry (w) , false, false, false, false, true)  //     ,    ,      ,    ,  TCACHE
    will_fire_stq_tag_retry (w) := lsu_sched(can_fire_stq_tag_retry (w) , false, false, false, false, true)  //     ,    ,      ,    ,  TCACHE
    } else {
      will_fire_ldq_tag_retry (w) := false.B
      will_fire_stq_tag_retry (w) := false.B
    }
    will_fire_load_wakeup   (w) := lsu_sched(can_fire_load_wakeup   (w) , false, true , true , false, false) //     , DC , LCAM1
    will_fire_store_commit  (w) := lsu_sched(can_fire_store_commit  (w) , false, true , false, false, false) //     , DC
    if (useMTE) {
    will_fire_stta_retry (w)    := lsu_sched(can_fire_stta_retry (w)    , true, false, false, true, false)  //  TLB ,    ,      , ROB ,  
    will_fire_stt_commit (w)    := lsu_sched(can_fire_stt_commit (w)    , false, false, true, false, true)  //      ,    , LCAM ,    ,  TCACHE
    } else {
      will_fire_stta_retry (w)    := false.B
      will_fire_stt_commit (w)    := false.B
    }


    assert(!(exe_req(w).valid && !(will_fire_load_incoming(w) || will_fire_stad_incoming(w) || will_fire_sta_incoming(w) || will_fire_std_incoming(w) || will_fire_sfence(w) || will_fire_stta_incoming(w))))

    when (will_fire_load_wakeup(w)) {
      block_load_mask(ldq_wakeup_idx)           := true.B
    } .elsewhen (will_fire_load_incoming(w)) {
      block_load_mask(exe_req(w).bits.uop.ldq_idx) := true.B
    } .elsewhen (will_fire_load_retry(w)) {
      block_load_mask(ldq_retry_idx)            := true.B
    }
    exe_tlb_valid(w) := !tlb_avail
  }
  assert((memWidth == 1).B ||
    (!(will_fire_sfence.reduce(_||_) && !will_fire_sfence.reduce(_&&_)) &&
     !will_fire_hella_incoming.reduce(_&&_) &&
     !will_fire_hella_wakeup.reduce(_&&_)   &&
     !will_fire_load_retry.reduce(_&&_)     &&
     !will_fire_sta_retry.reduce(_&&_)      &&
     !will_fire_store_commit.reduce(_&&_)   &&
     !will_fire_load_wakeup.reduce(_&&_)),
    "Some operations is proceeding down multiple pipes")

  require(memWidth <= 2)

  //--------------------------------------------
  // TLB Access

  assert(!(hella_state =/= h_ready && hella_req.cmd === rocket.M_SFENCE),
    "SFENCE through hella interface not supported")

  val exe_tlb_uop = widthMap(w =>
                    Mux(will_fire_load_incoming (w) ||
                        will_fire_stad_incoming (w) ||
                        will_fire_sta_incoming  (w) ||
                        will_fire_sfence        (w) ||
                        will_fire_stta_incoming (w)  , exe_req(w).bits.uop,
                    Mux(will_fire_load_retry    (w)  , ldq_retry_e.bits.uop,
                    Mux(will_fire_stta_retry    (w)  , stta_retry_e.bits.uop,
                    Mux(will_fire_sta_retry     (w)  , stq_retry_e.bits.uop,
                    Mux(will_fire_hella_incoming(w)  , NullMicroOp,
                                                       NullMicroOp))))))

  when (exe_tlb_uop(0).uses_stq && exe_tlb_uop(0).is_mte_tag_write) {
    assert(!exe_tlb_uop(0).ctrl.is_sta, "Cracked tag write?")
  }
  val exe_tlb_vaddr = widthMap(w =>
                    Mux(will_fire_load_incoming (w) ||
                        will_fire_stad_incoming (w) ||
                        will_fire_sta_incoming  (w) ||
                        will_fire_stta_incoming (w)  , exe_req(w).bits.addr,
                    Mux(will_fire_sfence        (w)  , exe_req(w).bits.sfence.bits.addr,
                    Mux(will_fire_load_retry    (w)  , ldq_retry_e.bits.addr.bits,
                    Mux(will_fire_stta_retry    (w)  , stta_retry_e.bits.addr.bits,
                    Mux(will_fire_sta_retry     (w)  , stq_retry_e.bits.addr.bits,
                    Mux(will_fire_hella_incoming(w)  , hella_req.addr,
                                                       0.U)))))))

  val ldst_vaddr_mte_tag = widthMap(w => {
    if (useMTE) {
      Mux(will_fire_load_incoming (w) ||
          will_fire_stad_incoming (w) ||
          will_fire_sta_incoming  (w)  , exe_req(w).bits.mte_tag.get,
      Mux(will_fire_load_retry    (w)  , ldq_retry_e.bits.addr_mte_tag.get,
      Mux(will_fire_stta_retry    (w)  , stta_retry_e.bits.addr_mte_tag.get,
      Mux(will_fire_sta_retry     (w)  , stq_retry_e.bits.addr_mte_tag.get,
                                         0.U))))
    } else {
      0.U
    }
  })

  val exe_sfence = WireInit((0.U).asTypeOf(Valid(new rocket.SFenceReq)))
  for (w <- 0 until memWidth) {
    when (will_fire_sfence(w)) {
      exe_sfence := exe_req(w).bits.sfence
    }
  }

  val exe_size   = widthMap(w =>
                   Mux(will_fire_load_incoming (w) ||
                       will_fire_stad_incoming (w) ||
                       will_fire_sta_incoming  (w) ||
                       will_fire_sfence        (w) ||
                       will_fire_load_retry    (w) ||
                       will_fire_sta_retry     (w)  , exe_tlb_uop(w).mem_size,
                   Mux(will_fire_hella_incoming(w)  , hella_req.size,
                                                      0.U)))
  val exe_cmd    = widthMap(w =>
                   Mux(will_fire_load_incoming (w) ||
                       will_fire_stad_incoming (w) ||
                       will_fire_sta_incoming  (w) ||
                       will_fire_sfence        (w) ||
                       will_fire_load_retry    (w) ||
                       will_fire_sta_retry     (w)  , exe_tlb_uop(w).mem_cmd,
                   Mux(will_fire_hella_incoming(w)  , hella_req.cmd,
                                                      0.U)))

  val exe_passthr= widthMap(w =>
                   Mux(will_fire_hella_incoming(w)  , hella_req.phys,
                                                      false.B))
  val exe_kill   = widthMap(w =>
                   Mux(will_fire_hella_incoming(w)  , io.hellacache.s1_kill,
                                                      false.B))
  for (w <- 0 until memWidth) {
    dtlb.io.req(w).valid            := exe_tlb_valid(w)
    dtlb.io.req(w).bits.vaddr       := exe_tlb_vaddr(w)
    dtlb.io.req(w).bits.size        := exe_size(w)
    dtlb.io.req(w).bits.cmd         := exe_cmd(w)
    dtlb.io.req(w).bits.passthrough := exe_passthr(w)
    dtlb.io.req(w).bits.v           := io.ptw.status.v
    dtlb.io.req(w).bits.prv         := io.ptw.status.prv
  }
  dtlb.io.kill                      := exe_kill.reduce(_||_)
  dtlb.io.sfence                    := exe_sfence

  // exceptions
  val ma_ld = widthMap(w => will_fire_load_incoming(w) && exe_req(w).bits.mxcpt.valid) // We get ma_ld in memaddrcalc
  val ma_st = widthMap(w => (will_fire_sta_incoming(w) || will_fire_stad_incoming(w) || will_fire_stta_incoming(w)) && exe_req(w).bits.mxcpt.valid) // We get ma_ld in memaddrcalc
  val pf_ld = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).pf.ld && exe_tlb_uop(w).uses_ldq)
  val pf_st = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).pf.st && exe_tlb_uop(w).uses_stq)
  val ae_ld = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).ae.ld && exe_tlb_uop(w).uses_ldq)
  val ae_st = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).ae.st && exe_tlb_uop(w).uses_stq)

  // TODO check for xcpt_if and verify that never happens on non-speculative instructions.
  val mem_xcpt_valids = RegNext(widthMap(w =>
                     (pf_ld(w) || pf_st(w) || ae_ld(w) || ae_st(w) || ma_ld(w) || ma_st(w)) &&
                     !io.core.exception &&
                     !IsKilledByBranch(io.core.brupdate, exe_tlb_uop(w))))
  val mem_xcpt_uops   = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, exe_tlb_uop(w))))
  val mem_xcpt_causes = RegNext(widthMap(w =>
    Mux(ma_ld(w), rocket.Causes.misaligned_load.U,
    Mux(ma_st(w), rocket.Causes.misaligned_store.U,
    Mux(pf_ld(w), rocket.Causes.load_page_fault.U,
    Mux(pf_st(w), rocket.Causes.store_page_fault.U,
    Mux(ae_ld(w), rocket.Causes.load_access.U,
                  rocket.Causes.store_access.U)))))))
  val mem_xcpt_vaddrs = RegNext(exe_tlb_vaddr)

  for (w <- 0 until memWidth) {
    assert (!(dtlb.io.req(w).valid && exe_tlb_uop(w).is_fence), "Fence is pretending to talk to the TLB")
    assert (!((will_fire_load_incoming(w) || will_fire_sta_incoming(w) || will_fire_stad_incoming(w)) &&
      exe_req(w).bits.mxcpt.valid && dtlb.io.req(w).valid &&
    !(exe_tlb_uop(w).ctrl.is_load || exe_tlb_uop(w).ctrl.is_sta)),
      "A uop that's not a load or store-address is throwing a memory exception.")
  }

  mem_xcpt_valid := mem_xcpt_valids.reduce(_||_)
  mem_xcpt_cause := mem_xcpt_causes(0)
  mem_xcpt_uop   := mem_xcpt_uops(0)
  mem_xcpt_vaddr := mem_xcpt_vaddrs(0)
  var xcpt_found = mem_xcpt_valids(0)
  var oldest_xcpt_rob_idx = mem_xcpt_uops(0).rob_idx
  for (w <- 1 until memWidth) {
    val is_older = WireInit(false.B)
    when (mem_xcpt_valids(w) &&
      (IsOlder(mem_xcpt_uops(w).rob_idx, oldest_xcpt_rob_idx, io.core.rob_head_idx) || !xcpt_found)) {
      is_older := true.B
      mem_xcpt_cause := mem_xcpt_causes(w)
      mem_xcpt_uop   := mem_xcpt_uops(w)
      mem_xcpt_vaddr := mem_xcpt_vaddrs(w)
    }
    xcpt_found = xcpt_found || mem_xcpt_valids(w)
    oldest_xcpt_rob_idx = Mux(is_older, mem_xcpt_uops(w).rob_idx, oldest_xcpt_rob_idx)
  }

  val exe_tlb_miss  = widthMap(w => dtlb.io.req(w).valid && (dtlb.io.resp(w).miss || !dtlb.io.req(w).ready))
  val exe_tlb_paddr = widthMap(w => Cat(dtlb.io.resp(w).paddr(paddrBits-1,corePgIdxBits),
                                        exe_tlb_vaddr(w)(corePgIdxBits-1,0)))
  val exe_tlb_uncacheable = widthMap(w => !(dtlb.io.resp(w).cacheable))

  for (w <- 0 until memWidth) {
    assert (exe_tlb_paddr(w) === dtlb.io.resp(w).paddr || exe_req(w).bits.sfence.valid, "[lsu] paddrs should match.")

    when (mem_xcpt_valids(w))
    {
      assert(RegNext(will_fire_load_incoming(w) || will_fire_stad_incoming(w) || will_fire_sta_incoming(w) ||
        will_fire_load_retry(w) || will_fire_sta_retry(w) || will_fire_stta_incoming(w) || will_fire_stta_retry(w)))
      // Technically only faulting AMOs need this
      assert(mem_xcpt_uops(w).uses_ldq ^ mem_xcpt_uops(w).uses_stq)
      when (mem_xcpt_uops(w).uses_ldq)
      {
        ldq(mem_xcpt_uops(w).ldq_idx).bits.uop.exception := true.B
      }
        .otherwise
      {
        stq(mem_xcpt_uops(w).stq_idx).bits.uop.exception := true.B
      }
    }
  }


  //--------------------------------------------
  // Tag Cache Access
  // Launch requests into the tag cache
  def ldq_dump() = {
    if (useMTE) {
      printf("[lsu] ---LDQ DUMP (tsc=%d)\nhead=%d, tail=%d\n", io.core.tsc_reg, ldq_head, ldq_tail)
      for (i <- 0 until numLdqEntries) {
        val ldq_e = ldq(i)
        val ldq_b = ldq_e.bits
        printf("[lsu] LD.Q %d, valid=%x, addr=%x (V=%d, virtual=%d), addr_mte_tag=%x, phys_mte_tag=%x (V=%d), executed=%d, phys_mte_tag_pending=%d, succeeded=%d, committed=%d, uopc=%d, pc=%x\n",
          i.U, ldq_e.valid, ldq_b.addr.bits, ldq_b.addr.valid, ldq_b.addr_is_virtual, ldq_b.addr_mte_tag.get, ldq_b.phys_mte_tag.get.bits, ldq_b.phys_mte_tag.get.valid, ldq_b.executed, ldq_b.phys_mte_tag_pending.get, ldq_b.succeeded, ldq_b.committed, ldq_b.uop.uopc, ldq_b.uop.debug_pc)
      }
      printf("[lsu] ---END\n")

    }
  }

  def stq_dump() = {
    if (useMTE) {
      printf("[lsu] ---STQ DUMP (tsc=%d)\nhead=%d, tail=%d, commit head=%d, execute head=%d\n", io.core.tsc_reg, stq_head, stq_tail, stq_commit_head, stq_execute_head)
      for (i <- 0 until numStqEntries) {
        val stq_e = stq(i)
        val stq_b = stq_e.bits
        printf("[lsu] ST.Q %d, valid=%x, addr=%x (V=%d, virtual=%d), addr_mte_tag=%x, phys_mte_tag=%x (V=%d), phys_mte_tag_pending=%d, commit_in_flight=%d, succeeded=%d, committed=%d, uopc=%d, pc=%x\n",
          i.U, stq_e.valid, stq_b.addr.bits, stq_b.addr.valid, stq_b.addr_is_virtual, stq_b.addr_mte_tag.get, stq_b.phys_mte_tag.get.bits, stq_b.phys_mte_tag.get.valid, stq_b.phys_mte_tag_pending.get, stq_b.commit_in_flight, stq_b.succeeded, stq_b.committed, stq_b.uop.uopc, stq_b.uop.debug_pc)
      }
      printf("[lsu] ---END\n")
    }
  }
  val tcache_addr = widthMap(w =>
                    /* Tag retries already cleared TLB */
                    Mux(will_fire_ldq_tag_retry (w)  , ldq_tag_retry_e.bits.addr,
                    Mux(will_fire_stq_tag_retry (w)  , stq_tag_retry_e.bits.addr,
                    Mux(will_fire_stt_commit    (w)  , stq_commit_e.bits.addr,
                    {
                      /* all others still need to clear TLB, so grab from there */
                      val v = Wire(Valid(UInt(coreMaxAddrBits.W)))
                      v.valid := !exe_tlb_miss(w)
                      v.bits := exe_tlb_paddr(w)
                      v
                    }))))

  val tcache_ldq_e = widthMap(w =>
                    Mux(will_fire_load_incoming (w)  , ldq_incoming_e(w),
                    Mux(will_fire_load_retry    (w)  , ldq_retry_e,
                    Mux(will_fire_ldq_tag_retry (w)  , ldq_tag_retry_e,
                    {
                      val v = Wire(Valid(new LDQEntry))
                      v.valid := false.B
                      v.bits := DontCare
                      v
                    }
  ))))

  val tcache_stq_e = widthMap(w =>
                    Mux(will_fire_stad_incoming (w) ||
                        will_fire_sta_incoming  (w)  , stq_incoming_e(w),
                    Mux(will_fire_sta_retry     (w)  , stq_retry_e,
                    Mux(will_fire_stq_tag_retry (w)  , stq_tag_retry_e,
                    Mux(will_fire_stt_commit    (w)  , stq_commit_e,
                    {
                      val v = Wire(Valid(new STQEntry))
                      v.valid := false.B
                      v.bits := DontCare
                      v
                    }
  )))))

  val tcache_uop = widthMap({w =>
    Mux(tcache_ldq_e(w).valid, tcache_ldq_e(w).bits.uop, tcache_stq_e(w).bits.uop)
  })

  val tcache_will_fire = widthMap(w =>
    will_fire_load_incoming (w) ||
    will_fire_stad_incoming (w) ||
    will_fire_sta_incoming  (w) ||
    will_fire_load_retry    (w) ||
    will_fire_sta_retry     (w) ||
    will_fire_ldq_tag_retry (w) ||
    will_fire_stq_tag_retry (w) ||
    will_fire_stt_commit    (w)
  )

  val tcache_is_incoming = widthMap(w =>
    will_fire_load_incoming (w) ||
    will_fire_stad_incoming (w) ||
    will_fire_sta_incoming  (w)
  )

  val tcache_did_fire = Wire(Bool())
  tcache_did_fire := false.B
  if (useMTE) {
    require(memWidth == 1, "Tag cache does not support width > 1")
    val tcacheIO = io.tcache.get
    val mteEnabled = io.core.mte_enabled.get
    for (w <- 0 until memWidth) {
      val req = tcacheIO.req
      val reqB = req.bits
      /*
      Fire a tcache request.
      If the address translation failed (i.e. this is an incoming request),
      we won't launch now but we can retry later.
      */
      when (tcache_will_fire(w) && tcache_addr(w).valid) {
        printf("[lsu] tcache_will_fire addr=%x, uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d, pc=%x (%x), st data=%x, ld_i %d, stad_i %d, sta_i %d, ld_r %d, sta_r %d, ld_t_r %d, st_t_r %d, stt_c %d [tsc=%d]\n", 
            tcache_addr(w).bits,
            tcache_uop(w).uses_ldq, tcache_uop(w).ldq_idx, tcache_uop(w).uses_stq, tcache_uop(w).stq_idx,
            tcache_uop(w).debug_pc,
            tcache_uop(w).debug_inst,
            tcache_stq_e(w).bits.data.bits,
            will_fire_load_incoming (w),
            will_fire_stad_incoming (w),
            will_fire_sta_incoming  (w),
            will_fire_load_retry    (w),
            will_fire_sta_retry     (w),
            will_fire_ldq_tag_retry (w),
            will_fire_stq_tag_retry (w),
            will_fire_stt_commit    (w),
            io.core.tsc_reg
        )
        assert((tcache_stq_e(w).valid || tcache_ldq_e(w).valid), 
          "No valid LDQ or STQ entry but we're firing on TCache?")
        assert(((tcache_stq_e(w).valid && !tcache_ldq_e(w).valid) ||
            (!tcache_stq_e(w).valid && tcache_ldq_e(w).valid)),
          "Firing TCache with both LDQ and STQ valid")

        /*
        We can only actually fire the request if the tcache is ready (i.e. not
        maxed on misses) and if the address translation completed (either due to
        a TLB hit this cycle or from a previous if this is a retry)
        */
        assert(tcache_addr(w).valid, "Attempting to fire invalid address?")
        reqB.address := tcache_addr(w).bits
        when (tcache_uop(w).uopc === uopMTE_STTI) {
          assert(tcache_stq_e(w).bits.committed, "Attempting to fire uncommitted tag write")
          reqB.requestType := TCacheRequestTypeEnum.WRITE
        } .otherwise {
          reqB.requestType := TCacheRequestTypeEnum.READ
        }
        when (tcache_uop(w).uopc === uopMTE_STTI) {
          reqB.data := tcache_stq_e(w).bits.data.bits(MTEConfig.tagBits - 1, 0)
        } .otherwise {
          reqB.data := DontCare
        }

        val is_fence = tcache_uop(w).is_fence || tcache_uop(w).is_fencei
        val can_fire = req.ready && tcache_addr(w).valid && 
          !IsKilledByBranch(io.core.brupdate, tcache_uop(w)) &&
          !is_fence &&
          /*
          Tag writes can only fire if they do not already have an in-flight/complete 
          commit operation. We can end up in this situation if, say, dmem nacks
          and we rewind but tcache continues okay.
          */
          (!tcache_uop(w).is_mte_tag_write || !(tcache_stq_e(w).bits.commit_in_flight || tcache_stq_e(w).bits.succeeded)) &&
          /* Do not interact with TCache if MTE is disabled */
          mteEnabled

        reqB.uop := tcache_uop(w)
        reqB.uop.br_mask := GetNewBrMask(io.core.brupdate, tcache_uop(w).br_mask)

        /* If we're good to go, launch it! */
        tcache_did_fire := can_fire
        req.valid := can_fire
        when (tcache_ldq_e(w).valid) {
          assert(!is_fence, "Fences should never travel down ldq")
	        val e = ldq(tcache_uop(w).ldq_idx)
          assert(!e.bits.phys_mte_tag_pending.get, "Already executed?")
          assert(!e.bits.phys_mte_tag.get.valid, "Trying to fire when tags already valid")
	        printf("[lsu] LDQ %d, tcache_can_fire=%d, addr=%x [tsc=%d]\n", tcache_uop(w).ldq_idx, can_fire, reqB.address, io.core.tsc_reg)
          when (mteEnabled) {
            e.bits.phys_mte_tag_pending.get := can_fire
          } .otherwise {
            /* If MTE is not enabled, just fake the tags */
            e.bits.phys_mte_tag_pending.get := false.B
            e.bits.phys_mte_tag.get.valid := true.B
            e.bits.phys_mte_tag.get.bits := io.core.mte_permissive_tag.get
          }
	        ldq_dump()
        } .elsewhen(tcache_stq_e(w).valid) {
	        val e = stq(tcache_uop(w).stq_idx)
	        printf("[lsu] STQ %x, tcache_can_fire=%d, addr=%x, type=%d, data=%x [tsc=%d]\n", tcache_uop(w).stq_idx, can_fire, reqB.address, reqB.requestType.asUInt, reqB.data, io.core.tsc_reg)

          when (tcache_uop(w).is_mte_tag_write) {
            val already_launched = e.bits.commit_in_flight || e.bits.succeeded
            /* Mark the commit in flight */
            assert(!can_fire || (can_fire && !already_launched), "Attempting to re-execute an executed tag write")
            when (can_fire) {
              e.bits.commit_in_flight := true.B
            }
            when (can_fire || already_launched || !mteEnabled) {
              /*
              Speculatively advance the execute head if we fired or just step
              over if we've already executed.
              We also fake fire tag writes (i.e. nop them) when they are executed
              with MTE disabled
              */
              stq_execute_head := WrapInc(stq_execute_head, numStqEntries)
            }

            when (!mteEnabled) {
              /* 
              If we're fake launching the tag write, we need to complete it too
              since we won't get a response from the cache
              */
              e.bits.succeeded := true.B
            }
          } .otherwise {
            assert(!e.bits.phys_mte_tag_pending.get, "Already executed?")
            assert(!e.bits.phys_mte_tag.get.valid, "Trying to fire when tags already valid")
            when (mteEnabled) {
              e.bits.phys_mte_tag_pending.get := can_fire
            } .otherwise {
              /* If MTE is not enabled, just fake the tags */
              e.bits.phys_mte_tag_pending.get := false.B
              e.bits.phys_mte_tag.get.valid := true.B
              e.bits.phys_mte_tag.get.bits := io.core.mte_permissive_tag.get
            }
          }

	        stq_dump()
        } /* otherwise should be unreachable, already asserted above */
      }.otherwise {
        req.valid := false.B
        reqB := DontCare
      }
    }


    /* Handle tcache responses */
    val resp = tcacheIO.resp
    val respB = tcacheIO.resp.bits
    when (resp.valid) {
      assert(io.core.mte_enabled.get, "TCache should not respond when MTE is disabled")
	    
      when (respB.uop.uses_ldq) {
        printf("[lsu] tcache response arrived LDQ %d, addr=%x, nack=%d, tag=%x [tsc=%d]\n", respB.uop.ldq_idx, ldq(respB.uop.ldq_idx).bits.addr.bits, respB.nack,respB.data, io.core.tsc_reg)
        val ldq_e = ldq(respB.uop.ldq_idx)
        assert(ldq_e.valid, "TCache response for invalid/freed entry")
        assert(ldq_e.bits.phys_mte_tag_pending.get, "TCache response for non-executed entry")

        /* Clear the pending bit since the request has arrived */
        ldq_e.bits.phys_mte_tag_pending.get := false.B
        /* 
        Set the tag from the response only if one isn't already set.
        The tag may have been set by a tag write forwarding.
        */
        val tag = ldq_e.bits.phys_mte_tag.get
        when (!tag.valid) {
          tag.valid := !respB.nack
          tag.bits := respB.data
        }
	      ldq_dump()
      }.elsewhen(respB.uop.uses_stq) {
        printf("[lsu] tcache response arrived STQ %d, addr=%x, nack=%d, tag=%x [tsc=%d]\n", respB.uop.stq_idx, stq(respB.uop.stq_idx).bits.addr.bits, respB.nack,respB.data, io.core.tsc_reg)
        val stq_e = stq(respB.uop.stq_idx)
        assert(stq_e.valid, "TCache response for invalid/freed entry")
        stq_e.bits.commit_in_flight := false.B

        when (stq_e.bits.uop.is_mte_tag_write) {
          /* 
          If the write nack'd, we'll retry since the commit head won't change
          unless the store succeeded.
          */
	        assert(stq_e.bits.commit_in_flight, "Response for rewound/unlaunched tag write operation?")
          assert(!stq_e.bits.succeeded, "Tag write already succeded?")
          stq_e.bits.succeeded := !respB.nack
          when (respB.nack) {
            /*
            When the TCache nacks a write, it gurentees that all younger writes
            (including the incoming one right now) will be killed. This allows
            us to safely rewind 
            */
            stq_execute_head_rewinds(0).valid := IsOlder(respB.uop.stq_idx, stq_execute_head, stq_head)
            stq_execute_head_rewinds(0).bits := respB.uop.stq_idx

            /* Kill all younger in-flight tag stores */
            for (i <- 0 until numStqEntries) {
              val stq_i_e = stq(i.U)
              when (IsOlder(respB.uop.stq_idx, i.U, stq_head) && stq_i_e.bits.uop.is_mte_tag_write) {
                stq_i_e.bits.commit_in_flight := false.B
                when (stq_i_e.valid && stq_i_e.bits.commit_in_flight) {
                  printf("[lsu] nack clearing tcache commit_in_flight STQ %d [tsc=%d]\n", i.U, io.core.tsc_reg)
                }
              }
            }
          }
        } .otherwise {
          assert(stq_e.bits.phys_mte_tag_pending.get, "TCache response for non-executed entry")

          /* Clear the pending bit since the request has arrived */
          stq_e.bits.phys_mte_tag_pending.get := false.B
          /* 
          Set the tag from the response only if one isn't already set.
          The tag may have been set by a tag write forwarding.
          */
          val tag = stq_e.bits.phys_mte_tag.get
          when (!tag.valid) {
            tag.valid := !respB.nack
            tag.bits := respB.data
          }
        }
	      stq_dump()
      }.otherwise {
        assert(false.B, "TCache should not issue LSU responses for untracked uops")
      }
    }
  }


  //------------------------------
  // Issue Someting to Memory
  //
  // A memory op can come from many different places
  // The address either was freshly translated, or we are
  // reading a physical address from the LDQ,STQ, or the HellaCache adapter


  // defaults
  io.dmem.brupdate         := io.core.brupdate
  io.dmem.exception      := io.core.exception
  io.dmem.rob_head_idx   := io.core.rob_head_idx
  io.dmem.rob_pnr_idx    := io.core.rob_pnr_idx

  val dmem_req = Wire(Vec(memWidth, Valid(new BoomDCacheReq)))
  io.dmem.req.valid := dmem_req.map(_.valid).reduce(_||_)
  io.dmem.req.bits  := dmem_req
  val dmem_req_fire = widthMap(w => dmem_req(w).valid && io.dmem.req.fire)

  val s0_executing_loads = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B)))


  for (w <- 0 until memWidth) {
    dmem_req(w).valid := false.B
    dmem_req(w).bits.uop   := NullMicroOp
    dmem_req(w).bits.addr  := 0.U
    dmem_req(w).bits.data  := 0.U
    dmem_req(w).bits.is_hella := false.B
    io.dmem.s1_kill(w) := false.B

    when (will_fire_load_incoming(w)) {
      dmem_req(w).valid      := !exe_tlb_miss(w) && !exe_tlb_uncacheable(w)
      dmem_req(w).bits.addr  := exe_tlb_paddr(w)
      dmem_req(w).bits.uop   := exe_tlb_uop(w)

      s0_executing_loads(ldq_incoming_idx(w)) := dmem_req_fire(w)
      assert(!ldq_incoming_e(w).bits.executed)
    } .elsewhen (will_fire_load_retry(w)) {
      dmem_req(w).valid      := !exe_tlb_miss(w) && !exe_tlb_uncacheable(w)
      dmem_req(w).bits.addr  := exe_tlb_paddr(w)
      dmem_req(w).bits.uop   := exe_tlb_uop(w)

      s0_executing_loads(ldq_retry_idx) := dmem_req_fire(w)
      assert(!ldq_retry_e.bits.executed)
    } .elsewhen (will_fire_store_commit(w)) {
      /* Don't refire if we have a valid in-flight/complete commit */
      val already_launched = stq_commit_e.bits.commit_in_flight || stq_commit_e.bits.succeeded
      dmem_req(w).valid         := !already_launched
      dmem_req(w).bits.addr     := stq_commit_e.bits.addr.bits
      dmem_req(w).bits.data     := (new freechips.rocketchip.rocket.StoreGen(
                                    stq_commit_e.bits.uop.mem_size, 0.U,
                                    stq_commit_e.bits.data.bits,
                                    coreDataBytes)).data
      dmem_req(w).bits.uop      := stq_commit_e.bits.uop

      when ((dmem_req_fire(w) || already_launched)) {
        stq_execute_head := WrapInc(stq_execute_head, numStqEntries)
      }

      when (dmem_req_fire(w)) {
        stq_commit_e.bits.commit_in_flight := true.B
      }
    } .elsewhen (will_fire_load_wakeup(w)) {
      dmem_req(w).valid      := true.B
      dmem_req(w).bits.addr  := ldq_wakeup_e.bits.addr.bits
      dmem_req(w).bits.uop   := ldq_wakeup_e.bits.uop

      s0_executing_loads(ldq_wakeup_idx) := dmem_req_fire(w)

      assert(!ldq_wakeup_e.bits.executed && !ldq_wakeup_e.bits.addr_is_virtual)
    } .elsewhen (will_fire_hella_incoming(w)) {
      assert(hella_state === h_s1)

      dmem_req(w).valid               := !io.hellacache.s1_kill && (!exe_tlb_miss(w) || hella_req.phys)
      dmem_req(w).bits.addr           := exe_tlb_paddr(w)
      dmem_req(w).bits.data           := (new freechips.rocketchip.rocket.StoreGen(
        hella_req.size, 0.U,
        io.hellacache.s1_data.data,
        coreDataBytes)).data
      printf("[lsu] will_fire_hella_incoming addr=%x, data=%x, cmd=%x [tsc=%d]\n", exe_tlb_paddr(w), io.hellacache.s1_data.data, hella_req.cmd, io.core.tsc_reg)
      dmem_req(w).bits.uop.mem_cmd    := hella_req.cmd
      dmem_req(w).bits.uop.mem_size   := hella_req.size
      dmem_req(w).bits.uop.mem_signed := hella_req.signed
      dmem_req(w).bits.is_hella       := true.B

      hella_paddr := exe_tlb_paddr(w)
    }
      .elsewhen (will_fire_hella_wakeup(w))
    {
      assert(hella_state === h_replay)
      dmem_req(w).valid               := true.B
      dmem_req(w).bits.addr           := hella_paddr
      dmem_req(w).bits.data           := (new freechips.rocketchip.rocket.StoreGen(
        hella_req.size, 0.U,
        hella_data.data,
        coreDataBytes)).data
      dmem_req(w).bits.uop.mem_cmd    := hella_req.cmd
      dmem_req(w).bits.uop.mem_size   := hella_req.size
      dmem_req(w).bits.uop.mem_signed := hella_req.signed
      dmem_req(w).bits.is_hella       := true.B
    }

    //-------------------------------------------------------------
    // Write Addr into the LAQ/SAQ
    when (will_fire_load_incoming(w) || will_fire_load_retry(w))
    {
      val ldq_idx = Mux(will_fire_load_incoming(w), ldq_incoming_idx(w), ldq_retry_idx)
      ldq(ldq_idx).bits.addr.valid          := true.B
      ldq(ldq_idx).bits.addr.bits           := Mux(exe_tlb_miss(w), exe_tlb_vaddr(w), exe_tlb_paddr(w))
      if (useMTE) {
        ldq(ldq_idx).bits.addr_mte_tag.get       := ldst_vaddr_mte_tag(w)
        val phys_mte_tag = ldq(ldq_idx).bits.phys_mte_tag.get
        phys_mte_tag.valid := false.B
        phys_mte_tag.bits := DontCare
      }
      when (!exe_tlb_miss(w)) {
        printf("[lsu] tlb translates V%x->P%x (ldq=%d) [tsc=%d]\n", exe_tlb_vaddr(w), exe_tlb_paddr(w), ldq_idx, io.core.tsc_reg)
      }
      ldq(ldq_idx).bits.uop.pdst            := exe_tlb_uop(w).pdst
      ldq(ldq_idx).bits.addr_is_virtual     := exe_tlb_miss(w)
      ldq(ldq_idx).bits.addr_is_uncacheable := exe_tlb_uncacheable(w) && !exe_tlb_miss(w)

      assert(!(will_fire_load_incoming(w) && ldq_incoming_e(w).bits.addr.valid),
        "[lsu] Incoming load is overwriting a valid address")
    }

    when (will_fire_sta_incoming(w) || will_fire_stad_incoming(w) || will_fire_sta_retry(w) ||
          will_fire_stta_incoming(w) || will_fire_stta_retry(w))
    {
      val stq_idx = 
        Mux(will_fire_sta_incoming(w) || will_fire_stad_incoming(w) || will_fire_stta_incoming(w), stq_incoming_idx(w), 
        Mux(will_fire_stta_retry(w), stta_retry_idx,
                                     stq_retry_idx))

      stq(stq_idx).bits.addr.valid := !pf_st(w) // Prevent AMOs from executing!
      stq(stq_idx).bits.addr.bits  := Mux(exe_tlb_miss(w), exe_tlb_vaddr(w), exe_tlb_paddr(w))
      if (useMTE) {
        stq(stq_idx).bits.addr_mte_tag.get := ldst_vaddr_mte_tag(w)
        val phys_mte_tag = stq(stq_idx).bits.phys_mte_tag.get
        phys_mte_tag.valid := false.B
        phys_mte_tag.bits := DontCare
      }
      when (!exe_tlb_miss(w)) {
        printf("[lsu] tlb translates V%x->P%x (stq=%d) [tsc=%d]\n", exe_tlb_vaddr(w), exe_tlb_paddr(w), stq_idx, io.core.tsc_reg)
      }
      stq(stq_idx).bits.uop.pdst   := exe_tlb_uop(w).pdst // Needed for AMOs
      stq(stq_idx).bits.addr_is_virtual := exe_tlb_miss(w)

      assert(!(will_fire_sta_incoming(w) && stq_incoming_e(w).bits.addr.valid),
        "[lsu] Incoming store is overwriting a valid address")

    }

    //-------------------------------------------------------------
    // Write data into the STQ
    if (w == 0)
      io.core.fp_stdata.ready := !will_fire_std_incoming(w) && !will_fire_stad_incoming(w) && !will_fire_stta_incoming(w)
    val fp_stdata_fire = io.core.fp_stdata.fire && (w == 0).B
    when (will_fire_std_incoming(w) || 
          will_fire_stad_incoming(w) ||
          /* STTA does not crack so data arrives here too */ 
          will_fire_stta_incoming(w) ||
          fp_stdata_fire)
    {
      val sidx = Mux(will_fire_std_incoming(w)  || 
                     will_fire_stad_incoming(w) ||
                     will_fire_stta_incoming(w),
                        stq_incoming_idx(w),
                        io.core.fp_stdata.bits.uop.stq_idx)
      stq(sidx).bits.data.valid := true.B
      stq(sidx).bits.data.bits  := Mux(will_fire_std_incoming(w)  || 
                                       will_fire_stad_incoming(w) ||
                                       will_fire_stta_incoming(w),
                                        exe_req(w).bits.data,
                                        io.core.fp_stdata.bits.data)
      assert(!(stq(sidx).bits.data.valid),
        "[lsu] Incoming store is overwriting a valid data entry")
    }
  }
  val will_fire_stdf_incoming = io.core.fp_stdata.fire
  require (xLen >= fLen) // for correct SDQ size

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Cache Access Cycle (Mem)
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Note the DCache may not have accepted our request

  val exe_req_killed = widthMap(w => IsKilledByBranch(io.core.brupdate, exe_req(w).bits.uop))
  val stdf_killed = IsKilledByBranch(io.core.brupdate, io.core.fp_stdata.bits.uop)

  val fired_load_incoming  = widthMap(w => RegNext(will_fire_load_incoming(w) && !exe_req_killed(w)))
  val fired_stad_incoming  = widthMap(w => RegNext(will_fire_stad_incoming(w) && !exe_req_killed(w)))
  val fired_sta_incoming   = widthMap(w => RegNext(will_fire_sta_incoming (w) && !exe_req_killed(w)))
  val fired_std_incoming   = widthMap(w => RegNext(will_fire_std_incoming (w) && !exe_req_killed(w)))
  val fired_stta_incoming  = widthMap(w => RegNext(will_fire_stta_incoming (w) && !exe_req_killed(w)))
  val fired_stdf_incoming  = RegNext(will_fire_stdf_incoming && !stdf_killed)
  val fired_sfence         = RegNext(will_fire_sfence)
  val fired_release        = RegNext(will_fire_release)
  val fired_load_retry     = widthMap(w => RegNext(will_fire_load_retry   (w) && !IsKilledByBranch(io.core.brupdate, ldq_retry_e.bits.uop)))
  val fired_sta_retry      = widthMap(w => RegNext(will_fire_sta_retry    (w) && !IsKilledByBranch(io.core.brupdate, stq_retry_e.bits.uop)))
  val fired_stta_retry     = widthMap(w => RegNext(will_fire_stta_retry   (w) && !IsKilledByBranch(io.core.brupdate, stta_retry_e.bits.uop)))

  val fired_store_commit   = RegNext(will_fire_store_commit)
  val fired_load_wakeup    = widthMap(w => RegNext(will_fire_load_wakeup  (w) && !IsKilledByBranch(io.core.brupdate, ldq_wakeup_e.bits.uop)))
  val fired_hella_incoming = RegNext(will_fire_hella_incoming)
  val fired_hella_wakeup   = RegNext(will_fire_hella_wakeup)
  require(!useMTE || memWidth == 1, "MTE expecting memWidth=1")
  val fired_stt_commit     = RegNext(will_fire_stt_commit(0) && tcache_did_fire)

  val mem_incoming_uop     = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, exe_req(w).bits.uop)))
  val mem_ldq_incoming_e   = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, ldq_incoming_e(w))))
  val mem_stq_incoming_e   = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, stq_incoming_e(w))))
  val mem_ldq_wakeup_e     = RegNext(UpdateBrMask(io.core.brupdate, ldq_wakeup_e))
  val mem_ldq_retry_e      = RegNext(UpdateBrMask(io.core.brupdate, ldq_retry_e))
  val mem_stq_retry_e      = RegNext(UpdateBrMask(io.core.brupdate, stq_retry_e))
  val mem_stta_retry_e     = RegNext(UpdateBrMask(io.core.brupdate, stta_retry_e))
  val mem_ldq_e            = widthMap(w =>
                             Mux(fired_load_incoming(w), mem_ldq_incoming_e(w),
                             Mux(fired_load_retry   (w), mem_ldq_retry_e,
                             Mux(fired_load_wakeup  (w), mem_ldq_wakeup_e, (0.U).asTypeOf(Valid(new LDQEntry))))))
  val mem_stq_e            = widthMap(w =>
                             Mux(fired_stad_incoming(w) ||
                                 fired_sta_incoming (w), mem_stq_incoming_e(w),
                             Mux(fired_sta_retry    (w), mem_stq_retry_e, (0.U).asTypeOf(Valid(new STQEntry)))))
  val mem_stdf_uop         = RegNext(UpdateBrMask(io.core.brupdate, io.core.fp_stdata.bits.uop))


  val mem_tlb_miss             = RegNext(exe_tlb_miss)
  val mem_tlb_uncacheable      = RegNext(exe_tlb_uncacheable)
  val mem_paddr                = RegNext(widthMap(w => dmem_req(w).bits.addr))

  val fired_tcache_address = useMTE.option(RegNext(io.tcache.get.req.bits.address))
  val fired_tcache_data = useMTE.option(RegNext(io.tcache.get.req.bits.data))
  val fired_tcache_sidx = useMTE.option(RegNext(stq_execute_head))
  val fired_tcache_previous_store_queue_head = useMTE.option(RegNext(stq_head))

  // Task 1: Clr ROB busy bit
  val clr_bsy_valid   = RegInit(widthMap(w => false.B))
  val clr_bsy_rob_idx = Reg(Vec(memWidth, UInt(robAddrSz.W)))
  val clr_bsy_brmask  = Reg(Vec(memWidth, UInt(maxBrCount.W)))

  for (w <- 0 until memWidth) {
    clr_bsy_valid   (w) := false.B
    clr_bsy_rob_idx (w) := 0.U
    clr_bsy_brmask  (w) := 0.U

    val uop = RegInit(NullMicroOp)
    when (fired_stad_incoming(w)) {
      clr_bsy_valid   (w) := mem_stq_incoming_e(w).valid           &&
                            !mem_tlb_miss(w)                       &&
                            !mem_stq_incoming_e(w).bits.uop.is_amo &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      clr_bsy_rob_idx (w) := mem_stq_incoming_e(w).bits.uop.rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      uop := mem_stq_incoming_e(w).bits.uop
    } .elsewhen (fired_sta_incoming(w)) {
      clr_bsy_valid   (w) := mem_stq_incoming_e(w).valid            &&
                             mem_stq_incoming_e(w).bits.data.valid  &&
                            !mem_tlb_miss(w)                        &&
                            !mem_stq_incoming_e(w).bits.uop.is_amo  &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      clr_bsy_rob_idx (w) := mem_stq_incoming_e(w).bits.uop.rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      uop := mem_stq_incoming_e(w).bits.uop
    } .elsewhen (fired_std_incoming(w)) {
      clr_bsy_valid   (w) := mem_stq_incoming_e(w).valid                 &&
                             mem_stq_incoming_e(w).bits.addr.valid       &&
                            !mem_stq_incoming_e(w).bits.addr_is_virtual  &&
                            !mem_stq_incoming_e(w).bits.uop.is_amo       &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      clr_bsy_rob_idx (w) := mem_stq_incoming_e(w).bits.uop.rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      uop := mem_stq_incoming_e(w).bits.uop
    } .elsewhen (fired_sfence(w)) {
      clr_bsy_valid   (w) := (w == 0).B // SFence proceeds down all paths, only allow one to clr the rob
      clr_bsy_rob_idx (w) := mem_incoming_uop(w).rob_idx
      uop := mem_incoming_uop(w)
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_incoming_uop(w))
    } .elsewhen (fired_sta_retry(w)) {
      clr_bsy_valid   (w) := mem_stq_retry_e.valid            &&
                             mem_stq_retry_e.bits.data.valid  &&
                            !mem_tlb_miss(w)                  &&
                            !mem_stq_retry_e.bits.uop.is_amo  &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_retry_e.bits.uop)
      clr_bsy_rob_idx (w) := mem_stq_retry_e.bits.uop.rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_retry_e.bits.uop)
      uop := mem_stq_retry_e.bits.uop
    } .elsewhen (fired_stta_incoming(w)) {
      clr_bsy_valid   (w) := mem_stq_incoming_e(w).valid         &&
                             !mem_tlb_miss(w)                    &&
                             !IsKilledByBranch(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      clr_bsy_rob_idx (w) := mem_stq_incoming_e(w).bits.uop.rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      uop := mem_stq_incoming_e(w).bits.uop
    } .elsewhen(fired_stta_retry(w)) {
      clr_bsy_valid   (w) := mem_stta_retry_e.valid            &&
                            !mem_tlb_miss(w)                   &&
                            !IsKilledByBranch(io.core.brupdate, mem_stta_retry_e.bits.uop)
      clr_bsy_rob_idx (w) := mem_stta_retry_e.bits.uop.rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stta_retry_e.bits.uop)
      uop := mem_stta_retry_e.bits.uop
    }

    io.core.clr_bsy(w).valid := clr_bsy_valid(w) &&
                               !IsKilledByBranch(io.core.brupdate, clr_bsy_brmask(w)) &&
                               !io.core.exception && !RegNext(io.core.exception) && !RegNext(RegNext(io.core.exception))
    io.core.clr_bsy(w).bits  := clr_bsy_rob_idx(w)

    when (clr_bsy_valid(w) &&
            !IsKilledByBranch(io.core.brupdate, clr_bsy_brmask(w)) &&
            !io.core.exception && !RegNext(io.core.exception) && !RegNext(RegNext(io.core.exception))) {
      printf("[lsu] clr_bsy uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d, pc=%x [tsc=%d]\n",
        uop.uses_ldq, uop.ldq_idx, uop.uses_stq, uop.stq_idx,
        uop.debug_pc, io.core.tsc_reg
      )
    }
  }

  val stdf_clr_bsy_valid   = RegInit(false.B)
  val stdf_clr_bsy_rob_idx = Reg(UInt(robAddrSz.W))
  val stdf_clr_bsy_brmask  = Reg(UInt(maxBrCount.W))
  stdf_clr_bsy_valid   := false.B
  stdf_clr_bsy_rob_idx := 0.U
  stdf_clr_bsy_brmask  := 0.U
  when (fired_stdf_incoming) {
    val s_idx = mem_stdf_uop.stq_idx
    stdf_clr_bsy_valid   := stq(s_idx).valid                 &&
                            stq(s_idx).bits.addr.valid       &&
                            !stq(s_idx).bits.addr_is_virtual &&
                            !stq(s_idx).bits.uop.is_amo      &&
                            !IsKilledByBranch(io.core.brupdate, mem_stdf_uop)
    stdf_clr_bsy_rob_idx := mem_stdf_uop.rob_idx
    stdf_clr_bsy_brmask  := GetNewBrMask(io.core.brupdate, mem_stdf_uop)
  }



  io.core.clr_bsy(memWidth).valid := stdf_clr_bsy_valid &&
                                    !IsKilledByBranch(io.core.brupdate, stdf_clr_bsy_brmask) &&
                                    !io.core.exception && !RegNext(io.core.exception) && !RegNext(RegNext(io.core.exception))
  io.core.clr_bsy(memWidth).bits  := stdf_clr_bsy_rob_idx



  // Task 2: Do LD-LD. ST-LD searches for ordering failures
  //         Do LD-ST search for forwarding opportunities
  // We have the opportunity to kill a request we sent last cycle. Use it wisely!

  // We translated a store last cycle
  val do_st_search = widthMap(w => (fired_stad_incoming(w) || fired_sta_incoming(w) || fired_sta_retry(w)) && !mem_tlb_miss(w))
  // We translated a load last cycle
  val do_ld_search = widthMap(w => ((fired_load_incoming(w) || fired_load_retry(w)) && !mem_tlb_miss(w)) ||
                     fired_load_wakeup(w))
  // We are making a local line visible to other harts
  val do_release_search = widthMap(w => fired_release(w))

  // Store addrs don't go to memory yet, get it from the TLB response
  // Load wakeups don't go through TLB, get it through memory
  // Load incoming and load retries go through both

  val lcam_addr  = widthMap(w => Mux(fired_stad_incoming(w) || fired_sta_incoming(w) || fired_sta_retry(w),RegNext(exe_tlb_paddr(w)),
                                 Mux(fired_release(w), RegNext(io.dmem.release.bits.address),
                                 Mux(fired_stt_commit, fired_tcache_address.getOrElse(0.U),
                                    mem_paddr(w)))))
  val lcam_uop   = widthMap(w => Mux(do_st_search(w), mem_stq_e(w).bits.uop,
                                 Mux(do_ld_search(w), mem_ldq_e(w).bits.uop, NullMicroOp)))

  val lcam_mask  = widthMap(w => GenByteMask(lcam_addr(w), lcam_uop(w).mem_size))
  val lcam_st_dep_mask = widthMap(w => mem_ldq_e(w).bits.st_dep_mask)
  val lcam_is_release = widthMap(w => fired_release(w))
  val lcam_ldq_idx  = widthMap(w =>
                      Mux(fired_load_incoming(w), mem_incoming_uop(w).ldq_idx,
                      Mux(fired_load_wakeup  (w), RegNext(ldq_wakeup_idx),
                      Mux(fired_load_retry   (w), RegNext(ldq_retry_idx), 0.U))))
  val lcam_stq_idx  = widthMap(w =>
                      Mux(fired_stad_incoming(w) ||
                          fired_sta_incoming (w), mem_incoming_uop(w).stq_idx,
                      Mux(fired_sta_retry    (w), RegNext(stq_retry_idx), 0.U)))

  val can_forward = WireInit(widthMap(w =>
    Mux(fired_load_incoming(w) || fired_load_retry(w), !mem_tlb_uncacheable(w),
      !ldq(lcam_ldq_idx(w)).bits.addr_is_uncacheable)))

  // Mask of stores which we conflict on address with
  val ldst_addr_matches    = WireInit(widthMap(w => VecInit((0 until numStqEntries).map(x=>false.B))))
  // Mask of stores which we can forward from
  val ldst_forward_matches = WireInit(widthMap(w => VecInit((0 until numStqEntries).map(x=>false.B))))

  val failed_loads     = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B))) // Loads which we will report as failures (throws a mini-exception)
  val nacking_loads    = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B))) // Loads which are being nacked by dcache in the next stage

  val s1_executing_loads = RegNext(s0_executing_loads)
  val s1_set_execute     = WireInit(s1_executing_loads)

  val mem_forward_valid   = Wire(Vec(memWidth, Bool()))
  val mem_forward_ldq_idx = lcam_ldq_idx
  val mem_forward_ld_addr = lcam_addr
  val mem_forward_stq_idx = Wire(Vec(memWidth, UInt(log2Ceil(numStqEntries).W)))

  val wb_forward_valid    = RegNext(mem_forward_valid)
  val wb_forward_ldq_idx  = RegNext(mem_forward_ldq_idx)
  val wb_forward_ld_addr  = RegNext(mem_forward_ld_addr)
  val wb_forward_stq_idx  = RegNext(mem_forward_stq_idx)

  for (i <- 0 until numLdqEntries) {
    val l_valid = ldq(i).valid
    val l_bits  = ldq(i).bits
    val l_addr  = ldq(i).bits.addr.bits
    val l_mask  = GenByteMask(l_addr, l_bits.uop.mem_size)

    val l_forwarders      = widthMap(w => wb_forward_valid(w) && wb_forward_ldq_idx(w) === i.U)
    val l_is_forwarding   = l_forwarders.reduce(_||_)
    val l_forward_stq_idx = Mux(l_is_forwarding, Mux1H(l_forwarders, wb_forward_stq_idx), l_bits.forward_stq_idx)

    val tag_granule_bits = log2Ceil(MTEConfig.taggingGranuleBytes)
    val tag_addr_matches = widthMap(w => lcam_addr(w) >> tag_granule_bits === l_addr >> tag_granule_bits)
    val block_addr_matches = widthMap(w => lcam_addr(w) >> blockOffBits === l_addr >> blockOffBits)
    val dword_addr_matches = widthMap(w => block_addr_matches(w) && lcam_addr(w)(blockOffBits-1,3) === l_addr(blockOffBits-1,3))
    val mask_match   = widthMap(w => (l_mask & lcam_mask(w)) === l_mask)
    val mask_overlap = widthMap(w => (l_mask & lcam_mask(w)).orR)

    // Searcher is a store
    for (w <- 0 until memWidth) {

      when (do_release_search(w) &&
            l_valid              &&
            l_bits.addr.valid    &&
            block_addr_matches(w)) {
        // This load has been observed, so if a younger load to the same address has not
        // executed yet, this load must be squashed
        ldq(i).bits.observed := true.B
      } .elsewhen (do_st_search(w)                                                                                                &&
                   l_valid                                                                                                        &&
                   l_bits.addr.valid                                                                                              &&
                   (l_bits.executed || l_bits.succeeded || l_is_forwarding)                                                       &&
                   !l_bits.addr_is_virtual                                                                                        &&
                   l_bits.st_dep_mask(lcam_stq_idx(w))                                                                            &&
                   dword_addr_matches(w)                                                                                          &&
                   mask_overlap(w)) {

        val forwarded_is_older = IsOlder(l_forward_stq_idx, lcam_stq_idx(w), l_bits.youngest_stq_idx)
        // We are older than this load, which overlapped us.
        when (!l_bits.forward_std_val || // If the load wasn't forwarded, it definitely failed
          ((l_forward_stq_idx =/= lcam_stq_idx(w)) && forwarded_is_older)) { // If the load forwarded from us, we might be ok
          ldq(i).bits.order_fail := true.B
          failed_loads(i)        := true.B
        }
      } .elsewhen (do_ld_search(w)            &&
                   l_valid                    &&
                   l_bits.addr.valid          &&
                   !l_bits.addr_is_virtual    &&
                   dword_addr_matches(w)      &&
                   mask_overlap(w)) {
        val searcher_is_older = IsOlder(lcam_ldq_idx(w), i.U, ldq_head)
        when (searcher_is_older) {
          when ((l_bits.executed || l_bits.succeeded || l_is_forwarding) &&
                !s1_executing_loads(i) && // If the load is proceeding in parallel we don't need to kill it
                l_bits.observed) {        // Its only a ordering failure if the cache line was observed between the younger load and us
            ldq(i).bits.order_fail := true.B
            failed_loads(i)        := true.B
          }
        } .elsewhen (lcam_ldq_idx(w) =/= i.U) {
          // The load is older, and either it hasn't executed, it was nacked, or it is ignoring its response
          // we need to kill ourselves, and prevent forwarding
          val older_nacked = nacking_loads(i) || RegNext(nacking_loads(i))
          when (!(l_bits.executed || l_bits.succeeded) || older_nacked) {
            s1_set_execute(lcam_ldq_idx(w))    := false.B
            io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
            can_forward(w)                     := false.B
          }
        }
      } .elsewhen (fired_stt_commit &&
                   l_valid &&
                   l_bits.addr.valid &&
                   !l_bits.addr_is_virtual &&
                   tag_addr_matches(w) &&
                   IsOlder(fired_tcache_sidx.getOrElse(0.U), l_bits.youngest_stq_idx, fired_tcache_previous_store_queue_head.getOrElse(0.U))) {
  /* 
  Store-to-load fixup forwarding for tags
  -------
  The general idea here is that since the tag cache gurentees coherency (i.e. a
  read reflects the most recent ack'd write) and because we fire writes into the
  cache in-order, we can trivially implement memory ordering by simply smearing
  writes forward to any younger tag loads which may have already executed.
  While tag writes can nack (thereby allowing later reads to fetch the old
  value), since we always retry tag commits until they work, we'll keep smearing
  and rewriting until we succesfully get one into the cache pipeline that will
  later be accepted.

  We place this after the initial tag write to one up any tags that may be 
  landing in the queues this cycle.
  */
        if (useMTE) {
          /* We're forwarding eligible! Overwrite the tag */
          l_bits.phys_mte_tag.get.valid := true.B
          l_bits.phys_mte_tag.get.bits := fired_tcache_data.get
          printf("[lsu] LCAM forwarding tag write to LDQ %d [tsc=%d]\n", i.U, io.core.tsc_reg)
        }
      }
    }
  }

  for (i <- 0 until numStqEntries) {
    val s_bits = stq(i).bits
    val s_addr = stq(i).bits.addr.bits
    val s_uop  = stq(i).bits.uop
    val dword_addr_matches = widthMap(w =>
                             ( stq(i).bits.addr.valid      &&
                              !stq(i).bits.addr_is_virtual &&
                              (s_addr(corePAddrBits-1,3) === lcam_addr(w)(corePAddrBits-1,3))))
    val tag_granule_bits = log2Ceil(MTEConfig.taggingGranuleBytes)
    val tag_addr_matches = widthMap(w => lcam_addr(w) >> tag_granule_bits === s_addr >> tag_granule_bits)

    val write_mask = GenByteMask(s_addr, s_uop.mem_size)
    for (w <- 0 until memWidth) {
      when (do_ld_search(w) && stq(i).valid && lcam_st_dep_mask(w)(i)) {
        when (((lcam_mask(w) & write_mask) === lcam_mask(w)) && !s_uop.is_fence && dword_addr_matches(w) && can_forward(w))
        {
          ldst_addr_matches(w)(i)            := true.B
          ldst_forward_matches(w)(i)         := true.B
          io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
          s1_set_execute(lcam_ldq_idx(w))    := false.B
        }
          .elsewhen (((lcam_mask(w) & write_mask) =/= 0.U) && dword_addr_matches(w))
        {
          ldst_addr_matches(w)(i)            := true.B
          io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
          s1_set_execute(lcam_ldq_idx(w))    := false.B
        }
          .elsewhen (s_uop.is_fence || s_uop.is_amo)
        {
          ldst_addr_matches(w)(i)            := true.B
          io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
          s1_set_execute(lcam_ldq_idx(w))    := false.B
        }
      } .elsewhen (fired_stt_commit && stq(i).valid &&
                     s_bits.addr.valid && !s_bits.addr_is_virtual &&
                     tag_addr_matches(w) &&
                     !s_uop.is_fence && !s_uop.is_mte_tag_write) {
          if (useMTE) {
            /* We're forwarding eligible! Overwrite the tag */
            s_bits.phys_mte_tag.get.valid := true.B
            s_bits.phys_mte_tag.get.bits := fired_tcache_data.get
            printf("[lsu] LCAM forwarding tag write to STQ %d [tsc=%d]\n", i.U, io.core.tsc_reg)
          }
      }
    }
  }

  // Set execute bit in LDQ
  for (i <- 0 until numLdqEntries) {
    when (s1_set_execute(i)) { ldq(i).bits.executed := true.B }
  }

  // Find the youngest store which the load is dependent on
  val forwarding_age_logic = Seq.fill(memWidth) { Module(new ForwardingAgeLogic(numStqEntries)) }
  for (w <- 0 until memWidth) {
    forwarding_age_logic(w).io.addr_matches    := ldst_addr_matches(w).asUInt
    forwarding_age_logic(w).io.youngest_st_idx := lcam_uop(w).stq_idx
  }
  val forwarding_idx = widthMap(w => forwarding_age_logic(w).io.forwarding_idx)

  // Forward if st-ld forwarding is possible from the writemask and loadmask
  mem_forward_valid       := widthMap(w =>
                                  (ldst_forward_matches(w)(forwarding_idx(w))        &&
                                 !IsKilledByBranch(io.core.brupdate, lcam_uop(w))    &&
                                 !io.core.exception && !RegNext(io.core.exception)))
  mem_forward_stq_idx     := forwarding_idx

  // Avoid deadlock with a 1-w LSU prioritizing load wakeups > store commits
  // On a 2W machine, load wakeups and store commits occupy separate pipelines,
  // so only add this logic for 1-w LSU
  if (memWidth == 1) {
    // Wakeups may repeatedly find a st->ld addr conflict and fail to forward,
    // repeated wakeups may block the store from ever committing
    // Disallow load wakeups 1 cycle after this happens to allow the stores to drain
    when (RegNext(ldst_addr_matches(0).reduce(_||_) && !mem_forward_valid(0))) {
      block_load_wakeup := true.B
    }

    // If stores remain blocked for 15 cycles, block load wakeups to get a store through
    val store_blocked_counter = Reg(UInt(4.W))
    when (will_fire_store_commit(0) || !can_fire_store_commit(0)) {
      store_blocked_counter := 0.U
    } .elsewhen (can_fire_store_commit(0) && !will_fire_store_commit(0)) {
      store_blocked_counter := Mux(store_blocked_counter === 15.U, 15.U, store_blocked_counter + 1.U)
    }
    when (store_blocked_counter === 15.U) {
      block_load_wakeup := true.B
    }
  }


  // Task 3: Clr unsafe bit in ROB for succesful translations
  //         Delay this a cycle to avoid going ahead of the exception broadcast
  //         The unsafe bit is cleared on the first translation, so no need to fire for load wakeups
  for (w <- 0 until memWidth) {
    io.core.clr_unsafe(w).valid := RegNext((do_st_search(w) || do_ld_search(w)) && !fired_load_wakeup(w)) && false.B
    io.core.clr_unsafe(w).bits  := RegNext(lcam_uop(w).rob_idx)
  }

  // detect which loads get marked as failures, but broadcast to the ROB the oldest failing load
  // TODO encapsulate this in an age-based  priority-encoder
  //   val l_idx = AgePriorityEncoder((Vec(Vec.tabulate(numLdqEntries)(i => failed_loads(i) && i.U >= laq_head)
  //   ++ failed_loads)).asUInt)
  val temp_bits = (VecInit(VecInit.tabulate(numLdqEntries)(i =>
    failed_loads(i) && i.U >= ldq_head) ++ failed_loads)).asUInt
  val l_idx = PriorityEncoder(temp_bits)

  // one exception port, but multiple causes!
  // - 1) the incoming store-address finds a faulting load (it is by definition younger)
  // - 2) the incoming load or store address is excepting. It must be older and thus takes precedent.
  val r_xcpt_valid = RegInit(false.B)
  val r_xcpt       = Reg(new Exception)

  val ld_xcpt_valid = failed_loads.reduce(_|_)
  val ld_xcpt_uop   = ldq(Mux(l_idx >= numLdqEntries.U, l_idx - numLdqEntries.U, l_idx)).bits.uop

  val use_mem_xcpt = (mem_xcpt_valid && IsOlder(mem_xcpt_uop.rob_idx, ld_xcpt_uop.rob_idx, io.core.rob_head_idx)) || !ld_xcpt_valid

  val xcpt_uop = Mux(use_mem_xcpt, mem_xcpt_uop, ld_xcpt_uop)

  r_xcpt_valid := (ld_xcpt_valid || mem_xcpt_valid) &&
                   !io.core.exception &&
                   !IsKilledByBranch(io.core.brupdate, xcpt_uop)
  r_xcpt.uop         := xcpt_uop
  r_xcpt.uop.br_mask := GetNewBrMask(io.core.brupdate, xcpt_uop)
  r_xcpt.cause       := Mux(use_mem_xcpt, mem_xcpt_cause, MINI_EXCEPTION_MEM_ORDERING)
  r_xcpt.badvaddr    := mem_xcpt_vaddr // TODO is there another register we can use instead?

  io.core.lxcpt.valid := r_xcpt_valid && !io.core.exception && !IsKilledByBranch(io.core.brupdate, r_xcpt.uop)
  io.core.lxcpt.bits  := r_xcpt

  // Task 4: Speculatively wakeup loads 1 cycle before they come back
  for (w <- 0 until memWidth) {
    io.core.spec_ld_wakeup(w).valid := enableFastLoadUse.B          &&
                                       fired_load_incoming(w)       &&
                                       !mem_incoming_uop(w).fp_val  &&
                                       mem_incoming_uop(w).pdst =/= 0.U
    io.core.spec_ld_wakeup(w).bits  := mem_incoming_uop(w).pdst
  }


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Writeback Cycle (St->Ld Forwarding Path)
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Handle Memory Responses and nacks
  //----------------------------------
  for (w <- 0 until memWidth) {
    io.core.exe(w).iresp.valid := false.B
    io.core.exe(w).fresp.valid := false.B
  }

  val dmem_resp_fired = WireInit(widthMap(w => false.B))

  for (w <- 0 until memWidth) {
    // Handle nacks
    when (io.dmem.nack(w).valid)
    {
      // We have to re-execute this!
      when (io.dmem.nack(w).bits.is_hella)
      {
        assert(hella_state === h_wait || hella_state === h_dead)
      }
        .elsewhen (io.dmem.nack(w).bits.uop.uses_ldq)
      {
        assert(ldq(io.dmem.nack(w).bits.uop.ldq_idx).bits.executed)
        ldq(io.dmem.nack(w).bits.uop.ldq_idx).bits.executed  := false.B
        nacking_loads(io.dmem.nack(w).bits.uop.ldq_idx) := true.B
      }
        .otherwise
      {
        assert(io.dmem.nack(w).bits.uop.uses_stq)
        printf("[lsu] dmem nack STQ %d [tsc=%d]\n", io.dmem.nack(w).bits.uop.stq_idx, io.core.tsc_reg)
        stq(io.dmem.nack(w).bits.uop.stq_idx).bits.commit_in_flight := false.B
        stq_execute_head_rewinds(1).valid := IsOlder(io.dmem.nack(w).bits.uop.stq_idx, stq_execute_head, stq_head)
        stq_execute_head_rewinds(1).bits := io.dmem.nack(w).bits.uop.stq_idx
        
        /* Kill all younger in-flight data stores */
        for (i <- 0 until numStqEntries) {
            val stq_i_e = stq(i.U)
            when (IsOlder(io.dmem.nack(w).bits.uop.stq_idx, i.U, stq_head) && !stq_i_e.bits.uop.is_mte_tag_write) {
              stq_i_e.bits.commit_in_flight := false.B
              when (stq_i_e.valid && stq_i_e.bits.commit_in_flight) {
                printf("[lsu] nack clearing dmem commit_in_flight STQ %d [tsc=%d]\n", i.U, io.core.tsc_reg)
              }
            }
          }
      }
    }
    // Handle the response
    when (io.dmem.resp(w).valid)
    {
      when (io.dmem.resp(w).bits.uop.uses_ldq)
    {
        assert(!io.dmem.resp(w).bits.is_hella)
        val ldq_idx = io.dmem.resp(w).bits.uop.ldq_idx
        val send_iresp = ldq(ldq_idx).bits.uop.dst_rtype === RT_FIX
        val send_fresp = ldq(ldq_idx).bits.uop.dst_rtype === RT_FLT
        // ldq_dump()
        // stq_dump()
        // printf("[lsu] dmem LDQ %d addr=%x\n", ldq_idx, ldq(ldq_idx).bits.addr.bits)

        io.core.exe(w).iresp.bits.uop  := ldq(ldq_idx).bits.uop
        io.core.exe(w).fresp.bits.uop  := ldq(ldq_idx).bits.uop
        io.core.exe(w).iresp.valid     := send_iresp
        io.core.exe(w).iresp.bits.data := io.dmem.resp(w).bits.data
        io.core.exe(w).fresp.valid     := send_fresp
        io.core.exe(w).fresp.bits.data := io.dmem.resp(w).bits.data

        assert(send_iresp ^ send_fresp)
        dmem_resp_fired(w) := true.B

        ldq(ldq_idx).bits.succeeded      := io.core.exe(w).iresp.valid || io.core.exe(w).fresp.valid
        ldq(ldq_idx).bits.debug_wb_data  := io.dmem.resp(w).bits.data
      }
        .elsewhen (io.dmem.resp(w).bits.uop.uses_stq)
      {
        assert(!io.dmem.resp(w).bits.is_hella)
        assert(!stq(io.dmem.resp(w).bits.uop.stq_idx).bits.uop.is_mte_tag_write, "Tag write travelled down dmem path?")
        stq(io.dmem.resp(w).bits.uop.stq_idx).bits.succeeded := true.B
        stq(io.dmem.resp(w).bits.uop.stq_idx).bits.commit_in_flight := false.B
        // ldq_dump()
        // stq_dump()
        printf("[lsu] dmem STQ %d addr=%x [tsc=%d]\n", io.dmem.resp(w).bits.uop.stq_idx, stq(io.dmem.resp(w).bits.uop.stq_idx).bits.addr.bits, io.core.tsc_reg)
        when (io.dmem.resp(w).bits.uop.is_amo) {
          dmem_resp_fired(w) := true.B
          io.core.exe(w).iresp.valid     := true.B
          io.core.exe(w).iresp.bits.uop  := stq(io.dmem.resp(w).bits.uop.stq_idx).bits.uop
          io.core.exe(w).iresp.bits.data := io.dmem.resp(w).bits.data

          stq(io.dmem.resp(w).bits.uop.stq_idx).bits.debug_wb_data := io.dmem.resp(w).bits.data
        }
      }
    }


    when (dmem_resp_fired(w) && wb_forward_valid(w))
    {
      // Twiddle thumbs. Can't forward because dcache response takes precedence
    }
      .elsewhen (!dmem_resp_fired(w) && wb_forward_valid(w))
    {
      val f_idx       = wb_forward_ldq_idx(w)
      val forward_uop = ldq(f_idx).bits.uop
      val stq_e       = stq(wb_forward_stq_idx(w))
      val data_ready  = stq_e.bits.data.valid
      val live        = !IsKilledByBranch(io.core.brupdate, forward_uop)
      val storegen = new freechips.rocketchip.rocket.StoreGen(
                                stq_e.bits.uop.mem_size, stq_e.bits.addr.bits,
                                stq_e.bits.data.bits, coreDataBytes)
      val loadgen  = new freechips.rocketchip.rocket.LoadGen(
                                forward_uop.mem_size, forward_uop.mem_signed,
                                wb_forward_ld_addr(w),
                                storegen.data, false.B, coreDataBytes)

      io.core.exe(w).iresp.valid := (forward_uop.dst_rtype === RT_FIX) && data_ready && live
      io.core.exe(w).fresp.valid := (forward_uop.dst_rtype === RT_FLT) && data_ready && live
      io.core.exe(w).iresp.bits.uop  := forward_uop
      io.core.exe(w).fresp.bits.uop  := forward_uop
      io.core.exe(w).iresp.bits.data := loadgen.data
      io.core.exe(w).fresp.bits.data := loadgen.data

      when (data_ready && live) {
        ldq(f_idx).bits.succeeded := data_ready
        ldq(f_idx).bits.forward_std_val := true.B
        ldq(f_idx).bits.forward_stq_idx := wb_forward_stq_idx(w)

        ldq(f_idx).bits.debug_wb_data   := loadgen.data
      }
    }
  }

  // Initially assume the speculative load wakeup failed
  io.core.ld_miss         := RegNext(io.core.spec_ld_wakeup.map(_.valid).reduce(_||_))
  val spec_ld_succeed = widthMap(w =>
    !RegNext(io.core.spec_ld_wakeup(w).valid) ||
    (io.core.exe(w).iresp.valid &&
      io.core.exe(w).iresp.bits.uop.ldq_idx === RegNext(mem_incoming_uop(w).ldq_idx)
    )
  ).reduce(_&&_)
  when (spec_ld_succeed) {
    io.core.ld_miss := false.B
  }


  //-------------------------------------------------------------
  // Kill speculated entries on branch mispredict
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Kill stores
  val st_brkilled_mask = Wire(Vec(numStqEntries, Bool()))
  for (i <- 0 until numStqEntries)
  {
    st_brkilled_mask(i) := false.B

    when (stq(i).valid)
    {
      stq(i).bits.uop.br_mask := GetNewBrMask(io.core.brupdate, stq(i).bits.uop.br_mask)

      when (IsKilledByBranch(io.core.brupdate, stq(i).bits.uop))
      {
        stq(i).valid           := false.B
        stq(i).bits.addr.valid := false.B
        stq(i).bits.data.valid := false.B
        if (useMTE) {
          stq(i).bits.phys_mte_tag.get.valid := false.B
          stq(i).bits.phys_mte_tag.get.bits := 0xb.U
          /* The TCache will kill any inflight requests so we can safely clear */
          stq(i).bits.phys_mte_tag_pending.get := false.B
        }
        printf("[lsu] STQ %d, addr=%x killed by branch [tsc=%d]\n", i.U, stq(i).bits.addr.bits, io.core.tsc_reg)
        st_brkilled_mask(i)    := true.B
      }
    }

    assert (!(IsKilledByBranch(io.core.brupdate, stq(i).bits.uop) && stq(i).valid && stq(i).bits.committed),
      "Branch is trying to clear a committed store.")
  }

  // Kill loads
  for (i <- 0 until numLdqEntries)
  {
    when (ldq(i).valid)
    {
      ldq(i).bits.uop.br_mask := GetNewBrMask(io.core.brupdate, ldq(i).bits.uop.br_mask)
      when (IsKilledByBranch(io.core.brupdate, ldq(i).bits.uop))
      {
        ldq(i).valid           := false.B
        ldq(i).bits.addr.valid := false.B
        if (useMTE) {
          ldq(i).bits.phys_mte_tag.get.valid := false.B
          ldq(i).bits.phys_mte_tag.get.bits := 0xb.U
          /* The TCache will kill any inflight requests so we can safely clear */
          ldq(i).bits.phys_mte_tag_pending.get := false.B
        }
        printf("[lsu] LDQ %d, addr=%x killed by branch [tsc=%d]\n", i.U, ldq(i).bits.addr.bits, io.core.tsc_reg)
      }
    }
  }

  //-------------------------------------------------------------
  when (io.core.brupdate.b2.mispredict && !io.core.exception)
  {
    stq_tail := io.core.brupdate.b2.uop.stq_idx
    ldq_tail := io.core.brupdate.b2.uop.ldq_idx
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // dequeue old entries on commit
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  var temp_stq_commit_head = stq_commit_head
  var temp_ldq_head        = ldq_head
  for (w <- 0 until coreWidth)
  {
    val commit_store = io.core.commit.valids(w) && io.core.commit.uops(w).uses_stq
    val commit_load  = io.core.commit.valids(w) && io.core.commit.uops(w).uses_ldq
    val idx = Mux(commit_store, io.core.commit.uops(w).stq_idx, io.core.commit.uops(w).ldq_idx)
    when (commit_store)
    {
      assert (stq(idx).valid, "[lsu] trying to commit an un-allocated store entry.")
      stq(idx).bits.committed := true.B
      ldq_dump()
      stq_dump()
    } .elsewhen (commit_load) {
      assert (ldq(idx).valid, "[lsu] trying to commit an un-allocated load entry.")
      ldq(idx).bits.committed := true.B
      ldq_dump()
      stq_dump()
    }

    if (MEMTRACE_PRINTF) {
      when (commit_store || commit_load) {
        val uop    = Mux(commit_store, stq(idx).bits.uop, ldq(idx).bits.uop)
        val addr   = Mux(commit_store, stq(idx).bits.addr.bits, ldq(idx).bits.addr.bits)
        val stdata = Mux(commit_store, stq(idx).bits.data.bits, 0.U)
        val wbdata = Mux(commit_store, stq(idx).bits.debug_wb_data, ldq(idx).bits.debug_wb_data)
        val addr_tag = Mux(commit_store, 
                            stq(idx).bits.addr_mte_tag.getOrElse(0.U), 
                            ldq(idx).bits.addr_mte_tag.getOrElse(0.U))
        printf("MT %x %x %x %x %x %x %x %x\n",
          io.core.tsc_reg, uop.uopc, uop.mem_cmd, uop.mem_size, addr, stdata, wbdata, addr_tag)
        printf("[LSU] core commit pc=%x, addr=%x, uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d [tsc=%d]\n", uop.debug_pc, addr,
          uop.uses_ldq, uop.ldq_idx, uop.uses_stq, uop.stq_idx, io.core.tsc_reg
        )
      }
    }
    
    temp_stq_commit_head = Mux(commit_store,
                               WrapInc(temp_stq_commit_head, numStqEntries),
                               temp_stq_commit_head)
  }
  stq_commit_head := temp_stq_commit_head

  // loads which have commited (and thus executed) as well as retrived tags can be cleared
  val ldq_head_dependent_tag_write_candidates_1h = (0 until numStqEntries).map{i =>
    Mux(ldq_head_e.bits.st_dep_mask(i),
        stq(i).bits.uop.is_mte_tag_write &&
          /* Block only on unforwarded tag writes */
          !(stq(i).bits.commit_in_flight || stq(i).bits.succeeded),
        false.B
    )
  }
  val ldq_head_has_dependent_tag_write_candidates = 
    ldq_head_dependent_tag_write_candidates_1h.reduce( _ || _)
  
  val clear_load = 
    ldq_head_e.valid && ldq_head_e.bits.committed && 
    ldq_head_e_mte_phys_completed &&
    /* 
    The entry cannot be cleared (even if we forwarded a valid tag) when an
    outstanding tag request exists in order to prevent a UaF
    */
    !ldq_head_e_mte_phys_mte_tag_pending &&
    !ldq_head_has_dependent_tag_write_candidates
  when (clear_load) {
    assert ((ldq_head_e.bits.executed || ldq_head_e.bits.forward_std_val) && ldq_head_e.bits.succeeded,
      "[lsu] trying to clear an un-executed load entry.")
    
    ldq_dump()

    val uop = ldq_head_e.bits.uop
    if (useMTE) {
      printf("[lsu] LDQ %d commit uopc=%x, mem_cmd=%x, addr=%x, addr_tag=%x, phys_tag=%x, ldq_head=%x, ldq_tail=%x [tsc=%d]\n", 
            ldq_head, uop.uopc, uop.mem_cmd, ldq_head_e.bits.addr.bits, ldq_head_e.bits.addr_mte_tag.get, ldq_head_e.bits.phys_mte_tag.get.bits,
            ldq_head, ldq_tail,
            io.core.tsc_reg
      )
    }


    ldq_head_e.valid                 := false.B
    ldq_head_e.bits.addr.valid       := false.B
    ldq_head_e.bits.executed         := false.B
    ldq_head_e.bits.succeeded        := false.B
    ldq_head_e.bits.order_fail       := false.B
    ldq_head_e.bits.forward_std_val  := false.B
    ldq_head_e.bits.committed        := false.B
    if (useMTE) {
        ldq_head_e.bits.phys_mte_tag.get.valid := false.B
        ldq_head_e.bits.phys_mte_tag.get.bits := 0xa.U
        ldq_head_e.bits.phys_mte_tag_pending.get := false.B
    }

    ldq_head :=  WrapInc(ldq_head, numLdqEntries)
  }

  // store has been committed AND successfully sent data to memory
  val stq_head_e = stq(stq_head)
  val stq_head_e_mte_phys_completed = {
    if (!useMTE) {
      true.B
    } else {
      stq_head_e.bits.phys_mte_tag.get.valid
    }
  }
  val stq_head_e_mte_phys_mte_tag_pending = stq_head_e.bits.phys_mte_tag_pending.getOrElse(false.B)

  when (stq_head_e.valid && stq_head_e.bits.committed)
  {
    when (stq_head_e.bits.uop.is_fence && !io.dmem.ordered) {
      io.dmem.force_order := true.B
      store_needs_order   := true.B
    }
    when (stq_head_e.bits.uop.is_fence) {
      clear_store := io.dmem.ordered
    } .elsewhen (stq_head_e.bits.uop.is_mte_tag_write) {
      /* 
      Tag writes to not need to wait for physical tag fetch since they are not
      tagged
      */
      clear_store := stq_head_e.bits.succeeded
    } .otherwise {
      /* Other stq entry, i.e. a regular store */
      clear_store := stq_head_e.bits.succeeded && 
        stq_head_e_mte_phys_completed &&
        /* 
        The entry cannot be cleared (even if we forwarded a valid tag) when an
        outstanding tag request exists in order to prevent a UaF
        */
        !stq_head_e_mte_phys_mte_tag_pending
    }
  }

  when (clear_store)
  {
    val uop = stq_head_e.bits.uop

    stq_head_e.valid           := false.B
    stq_head_e.bits.addr.valid := false.B
    stq_head_e.bits.data.valid := false.B
    stq_head_e.bits.succeeded  := false.B
    stq_head_e.bits.committed  := false.B
    stq_head_e.bits.commit_in_flight := false.B
    if (useMTE) {
        stq_head_e.bits.phys_mte_tag.get.valid := false.B
        stq_head_e.bits.phys_mte_tag.get.bits := 0xa.U
        stq_head_e.bits.phys_mte_tag_pending.get := false.B
        printf("[lsu] STQ %x commit uopc=%x, mem_cmd=%x, addr=%x, addr_tag=%x, phys_tag=%x, stq_head=%x, stq_tail=%x [tsc=%d]\n", 
                  stq_head, uop.uopc, uop.mem_cmd, stq_head_e.bits.addr.bits, stq_head_e.bits.addr_mte_tag.get, stq_head_e.bits.phys_mte_tag.get.bits,
                  stq_head, stq_tail, io.core.tsc_reg
            )
    }

    stq_head := WrapInc(stq_head, numStqEntries)
    when (stq_head_e.bits.uop.is_fence ||
      /*
      This is a weird quirk of the way we handle dmem and tcache nacks.
      If an older dmem store nacks after a younger tcache stt completes, we
      don't kill the stt and instead leave it in the stq as completed and just
      don't relaunch it (or vice versa).
      
      When such an entry is seen by the firing logic, it simply advances the 
      execute head. Notably, however, is that if the entry is the stq head and 
      it completed previously, it is store eligible. This is a problem if the 
      desired cache is not ready and cannot advance the pointer as it means 
      execute head begins to trail. Fix this up.
      */
          stq_head === stq_execute_head)
    {
      stq_execute_head := WrapInc(stq_execute_head, numStqEntries)
    }
  }

  /* 
  Handle store queue execute rewinds 
  We put this last so that it supercedes any similtaneous advances 
  */
  val stq_execute_head_rewind = stq_execute_head_rewinds.reduce({
    (a, b) =>
    Mux(!a.valid, b,
    Mux(!b.valid, a,
    Mux(IsOlder(a.bits, b.bits, stq_head), a,
                                           b)))
  })

  when (stq_execute_head_rewind.valid) {
    stq_execute_head := stq_execute_head_rewind.bits
  }

  // -----------------------
  // Hellacache interface
  // We need to time things like a HellaCache would
  io.hellacache.req.ready := false.B
  io.hellacache.s2_nack   := false.B
  io.hellacache.s2_xcpt   := (0.U).asTypeOf(new rocket.HellaCacheExceptions)
  io.hellacache.resp.valid := false.B
  when (hella_state === h_ready) {
    io.hellacache.req.ready := true.B
    when (io.hellacache.req.fire) {
      hella_req   := io.hellacache.req.bits
      hella_state := h_s1
    }
  } .elsewhen (hella_state === h_s1) {
    can_fire_hella_incoming(memWidth-1) := true.B

    hella_data := io.hellacache.s1_data
    when (hella_req.no_xcpt) {
      hella_xcpt := dtlb.io.resp(memWidth-1)
    } .otherwise {
      /* XXX: no_excpt === false apparently is supposed to SUPRESS exceptions */
      val no_exceptions = Wire(new rocket.AlignmentExceptions)
      no_exceptions.ld := false.B
      no_exceptions.st := false.B
      hella_xcpt.ma := no_exceptions
      hella_xcpt.pf := no_exceptions
      hella_xcpt.gf := no_exceptions
      hella_xcpt.ae := no_exceptions
    }

    when (io.hellacache.s1_kill) {
      when (will_fire_hella_incoming(memWidth-1) && dmem_req_fire(memWidth-1)) {
        hella_state := h_dead
      } .otherwise {
        hella_state := h_ready
      }
    } .elsewhen (will_fire_hella_incoming(memWidth-1) && dmem_req_fire(memWidth-1)) {
      hella_state := h_s2
    } .otherwise {
      hella_state := h_s2_nack
    }
  } .elsewhen (hella_state === h_s2_nack) {
    io.hellacache.s2_nack := true.B
    hella_state := h_ready
  } .elsewhen (hella_state === h_s2) {
    io.hellacache.s2_xcpt := hella_xcpt
    when (io.hellacache.s2_kill || hella_xcpt.asUInt =/= 0.U) {
      printf("[lsu] killing hellacache s2 request <addr=%x, size=%d, tag=%d>, s2_kill=%d, ma=%d, %d, pf=%d, %d, gf=%d, %d, ae=%d, %d [tsc=%d]",
              hella_req.addr, hella_req.size, hella_req.tag,
              io.hellacache.s2_kill,
              hella_xcpt.ma.ld, hella_xcpt.ma.st,
              hella_xcpt.pf.ld, hella_xcpt.pf.st,
              hella_xcpt.gf.ld, hella_xcpt.gf.st,
              hella_xcpt.ae.ld, hella_xcpt.ae.st,
              io.core.tsc_reg
        )
      hella_state := h_dead
    } .otherwise {
      hella_state := h_wait
    }
  } .elsewhen (hella_state === h_wait) {
    for (w <- 0 until memWidth) {
      when (io.dmem.resp(w).valid && io.dmem.resp(w).bits.is_hella) {
        hella_state := h_ready
        printf("[lsu] hella dmem response addr=%x, dmem_data=%x [tsc=%d]\n", hella_req.addr, io.dmem.resp(w).bits.data, io.core.tsc_reg)
        io.hellacache.resp.valid       := true.B
        io.hellacache.resp.bits.addr   := hella_req.addr
        io.hellacache.resp.bits.tag    := hella_req.tag
        io.hellacache.resp.bits.cmd    := hella_req.cmd
        io.hellacache.resp.bits.signed := hella_req.signed
        io.hellacache.resp.bits.size   := hella_req.size
        io.hellacache.resp.bits.data   := io.dmem.resp(w).bits.data
      } .elsewhen (io.dmem.nack(w).valid && io.dmem.nack(w).bits.is_hella) {
        printf("[lsu] hella dmem nack addr=%x [tsc=%d]\n", hella_req.addr, io.core.tsc_reg)
        hella_state := h_replay
      }
    }
  } .elsewhen (hella_state === h_replay) {
    can_fire_hella_wakeup(memWidth-1) := true.B

    when (will_fire_hella_wakeup(memWidth-1) && dmem_req_fire(memWidth-1)) {
      hella_state := h_wait
    }
  } .elsewhen (hella_state === h_dead) {
    for (w <- 0 until memWidth) {
      when (io.dmem.resp(w).valid && io.dmem.resp(w).bits.is_hella) {
        hella_state := h_ready
      }
    }
  }

  //-------------------------------------------------------------
  // Exception / Reset

  // for the live_store_mask, need to kill stores that haven't been committed
  val st_exc_killed_mask = WireInit(VecInit((0 until numStqEntries).map(x=>false.B)))

  when (reset.asBool || io.core.exception)
  {
    ldq_head := 0.U
    ldq_tail := 0.U

    when (reset.asBool)
    {
      stq_head := 0.U
      stq_tail := 0.U
      stq_commit_head  := 0.U
      stq_execute_head := 0.U

      for (i <- 0 until numStqEntries)
      {
        stq(i).valid           := false.B
        stq(i).bits.addr.valid := false.B
        stq(i).bits.data.valid := false.B
        stq(i).bits.succeeded  := false.B
        stq(i).bits.committed  := false.B
        stq(i).bits.commit_in_flight  := false.B
        if (useMTE) {
          stq(i).bits.phys_mte_tag.get.valid := false.B
          stq(i).bits.phys_mte_tag.get.bits := 0xe.U
          stq(i).bits.phys_mte_tag_pending.get := false.B
        }
        stq(i).bits.uop        := NullMicroOp
      }
    }
      .otherwise // exception
    {
      stq_tail := stq_commit_head
      printf("[lsu] exception! killing loads and stores [tsc=%d]\n", io.core.tsc_reg)
      for (i <- 0 until numStqEntries)
      {
        when (!stq(i).bits.committed && !stq(i).bits.succeeded)
        {
          stq(i).valid           := false.B
          stq(i).bits.addr.valid := false.B
          stq(i).bits.data.valid := false.B
          stq(i).bits.succeeded  := false.B
          stq(i).bits.committed  := false.B
          stq(i).bits.commit_in_flight  := false.B
          if (useMTE) {
            stq(i).bits.phys_mte_tag.get.valid := false.B
            stq(i).bits.phys_mte_tag.get.bits := 0xe.U
            /* TCache will kill in-flight requests, okay to clear */
            stq(i).bits.phys_mte_tag_pending.get := false.B
          }
          st_exc_killed_mask(i)  := true.B
        }
      }
    }

    //XXX: This design means that we drop some tag checks on an exception
    for (i <- 0 until numLdqEntries)
    {
      ldq(i).valid           := false.B
      ldq(i).bits.addr.valid := false.B
      ldq(i).bits.executed   := false.B
      ldq(i).bits.succeeded  := false.B
      ldq(i).bits.order_fail := false.B
      ldq(i).bits.forward_std_val := false.B
      ldq(i).bits.committed := false.B
      if (useMTE) {
          ldq(i).bits.phys_mte_tag.get.valid := false.B
          ldq(i).bits.phys_mte_tag.get.bits := 0xe.U
          /* TCache will kill in-flight requests, okay to clear */
          ldq(i).bits.phys_mte_tag_pending.get := false.B
      }
    }
  }

  if (useMTE) {
    /* 
    The TCache needs to know what stores are being killed as we launch tag
    reads speculatively for stores and do not inform it when a store commits.
    Without this information, it cannot correctly kill in-flight tag operations.
    */
    io.tcache.get.st_exc_killed_mask := st_exc_killed_mask
    io.tcache.get.stq_head_idx := stq_head
  }

  //-------------------------------------------------------------
  // Live Store Mask
  // track a bit-array of stores that are alive
  // (could maybe be re-produced from the stq_head/stq_tail, but need to know include spec_killed entries)

  // TODO is this the most efficient way to compute the live store mask?
  live_store_mask := next_live_store_mask &
                    ~(st_brkilled_mask.asUInt) &
                    ~(st_exc_killed_mask.asUInt)


  /*
  Generate MTE Faults
  -------------------
  Since we presently only support async mode and leave tag check to the absolute
  last possible moment (when entries are being cleared).
  At this point, we know that the tag at stq_head is at its final value as it is
  the oldest entry in the store queue (and thus there are no remaining tag writes
  which could overwrite it). The same is not true for the head of the load queue
  however because there may be 

  Unless we use LCAM to do store to load forwarding on each load (rather than the
  store forwarding on commit), we have to clear in order


  We can do something kinda evil--we can stall load clears until st_dep_mask is
  free of any tag writes
    * This is safe because the queues are ordered and so the store queue is
      guraneteed to make progress even with a full and blocked load queue.
    * This is likely to give very bad performance for compiler instrumented
      tagging as there will be many possible dependent tag stores
  
  Could add a new task for loads which is just a tag load searcher.
    * Once a load is committed, it can search for older tag writes using 
      LCAM. Once this process is complete, it can safely clear.
    * Once a store 
  */

  if (useMTE) {
    val pkt = io.core.mte_fault_packet.get
    val permissive_tag = io.core.mte_permissive_tag.get
    val cleared_store_older = IsOlder(stq_head, ldq_head_e.bits.youngest_stq_idx, stq_head)
    dontTouch(pkt)
    dontTouch(cleared_store_older)
    dontTouch(clear_load)
    dontTouch(clear_store)
    val clear_load_valid = clear_load && ldq_head_e.valid && 
      !io.core.exception && 
      !(ldq_head_e.bits.uop.is_mte_tag_write || ldq_head_e.bits.uop.is_fence || ldq_head_e.bits.uop.is_fencei)
    val clear_store_valid = clear_store && stq_head_e.valid && 
      !io.core.exception &&
      !(stq_head_e.bits.uop.is_mte_tag_write || stq_head_e.bits.uop.is_fence || stq_head_e.bits.uop.is_fencei)
    dontTouch(clear_load_valid)
    dontTouch(clear_store_valid)

    pkt.valid := false.B
    pkt.bits := DontCare
    when (clear_load_valid) {
      val addr_tag = ldq_head_e.bits.addr_mte_tag.get
      val phys_tag = ldq_head_e.bits.phys_mte_tag.get.bits
      when (addr_tag =/= phys_tag && phys_tag =/= permissive_tag) {
        pkt.valid := true.B
        pkt.bits.address_tag := addr_tag
        pkt.bits.physical_tag := phys_tag
        pkt.bits.is_load := true.B
        pkt.bits.faulting_address := ldq_head_e.bits.addr.bits
        pkt.bits.mem_size := ldq_head_e.bits.uop.mem_size
        pkt.bits.uop := ldq_head_e.bits.uop
      }
    }

    when (clear_store_valid) {
      val addr_tag = stq_head_e.bits.addr_mte_tag.get
      val phys_tag = stq_head_e.bits.phys_mte_tag.get.bits
      when (addr_tag =/= phys_tag && phys_tag =/= permissive_tag &&
            (!clear_load_valid || cleared_store_older)) {
        pkt.valid := true.B
        pkt.bits.address_tag := addr_tag
        pkt.bits.physical_tag := phys_tag
        pkt.bits.is_load := false.B
        pkt.bits.faulting_address := stq_head_e.bits.addr.bits
        pkt.bits.mem_size := stq_head_e.bits.uop.mem_size
        pkt.bits.uop := stq_head_e.bits.uop
      }
    }
  }

}

/**
 * Object to take an address and generate an 8-bit mask of which bytes within a
 * double-word.
 */
object GenByteMask
{
   def apply(addr: UInt, size: UInt): UInt =
   {
      val mask = Wire(UInt(8.W))
      mask := MuxCase(255.U(8.W), Array(
                   (size === 0.U) -> (1.U(8.W) << addr(2,0)),
                   (size === 1.U) -> (3.U(8.W) << (addr(2,1) << 1.U)),
                   (size === 2.U) -> Mux(addr(2), 240.U(8.W), 15.U(8.W)),
                   (size === 3.U) -> 255.U(8.W)))
      mask
   }
}

/**
 * ...
 */
class ForwardingAgeLogic(num_entries: Int)(implicit p: Parameters) extends BoomModule()(p)
{
   val io = IO(new Bundle
   {
      val addr_matches    = Input(UInt(num_entries.W)) // bit vector of addresses that match
                                                       // between the load and the SAQ
      val youngest_st_idx = Input(UInt(stqAddrSz.W)) // needed to get "age"

      val forwarding_val  = Output(Bool())
      val forwarding_idx  = Output(UInt(stqAddrSz.W))
   })

   // generating mask that zeroes out anything younger than tail
   val age_mask = Wire(Vec(num_entries, Bool()))
   for (i <- 0 until num_entries)
   {
      age_mask(i) := true.B
      when (i.U >= io.youngest_st_idx) // currently the tail points PAST last store, so use >=
      {
         age_mask(i) := false.B
      }
   }

   // Priority encoder with moving tail: double length
   val matches = Wire(UInt((2*num_entries).W))
   matches := Cat(io.addr_matches & age_mask.asUInt,
                  io.addr_matches)

   val found_match = Wire(Bool())
   found_match       := false.B
   io.forwarding_idx := 0.U

   // look for youngest, approach from the oldest side, let the last one found stick
   for (i <- 0 until (2*num_entries))
   {
      when (matches(i))
      {
         found_match := true.B
         io.forwarding_idx := (i % num_entries).U
      }
   }

   io.forwarding_val := found_match
}
