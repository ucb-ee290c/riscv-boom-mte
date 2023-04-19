package lsu

import chisel3._
import chisel3.util._
import chisel3.util.random._
import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.rocket.{HellaCacheIO, SimpleHellaCacheIF}
import boom.common._
import boom.lsu._
import boom.exu.{BrUpdateInfo}
import boom.util.{IsKilledByBranch, GetNewBrMask}
import freechips.rocketchip.util.DescribedSRAM
import chisel3.experimental.ChiselEnum
import freechips.rocketchip.tile.HasCoreParameters
import freechips.rocketchip.rocket.PRV
import freechips.rocketchip.util.RandomReplacement
import freechips.rocketchip.util.UIntToOH1
import boom.exu.BrUpdateMasks
import java.net.Authenticator.RequestorType

case class TCacheParams(
    // /**
    //   * The number of fully associative lines in the cache
    //   */
    // nEntries:Int = 8,

    // /** The number of bytes per cache entry/line */
    // entrySizeBytes:Int = 8,

    /**
     * The number of sets.
     * Note that nSets * blockSizeBytes must not be greater than L1D's block size
     * This is because our implementation forces different offsets of the same
     * L1D block into the same set, thereby allowing a single L1D probe to be
     * handled in a single cycle. This is a similar concept to VIPT caches where
     * the "direct map" portion of the tag cannot exceed the size of a page.
     * 
     * For example, with 64-byte block a valid (maximal) configuration could be
     * 8 sets with 8 byte blocks.
     */
    nSets:Int = 8,
    nWays:Int = 2,
    /* Must be fixed at <= coreDataBits when using BOOM L1D */
    blockSizeBytes:Int = 8,
    rowBits:Int = 64,

    /** The number of active, outstanding requests the TCache can have to NLC */
    nMSHRs:Int = 2
)

trait HasTCacheParameters extends HasCoreParameters {
    implicit val tcacheParams: TCacheParams
    require(MTEConfig.tagBits <= 8, "TCache assumes tags are smaller than a byte")
    val mteTagsPerByte = (8 / MTEConfig.tagBits)
    val mteTagsPerByteBits = log2Ceil(mteTagsPerByte)
    def mteTagsPerBlock = (tcacheParams.blockSizeBytes * 8) / MTEConfig.tagBits
    def mteTagsPerBlockBits = log2Ceil(mteTagsPerBlock)
    def cacheOffsetBits = log2Ceil(tcacheParams.blockSizeBytes)
    def cacheOffsetOff = 0
    def cacheIndexBits = log2Ceil(tcacheParams.nSets)
    def cacheIndexOff = cacheOffsetOff + cacheOffsetBits
    def cacheTagBits = coreMaxAddrBits - cacheIndexBits - cacheOffsetBits
    def cacheTagOff = cacheIndexOff + cacheIndexBits
    def nWaysBits = log2Ceil(tcacheParams.nWays)
    def nSetsBits = log2Ceil(tcacheParams.nSets)
}

// object TCacheLineStateEnum extends ChiselEnum {
//     val INVALID, READY, WRITEBACK = Value
// }

abstract class TCacheModule(implicit p: Parameters, tcacheParams: TCacheParams)
    extends BoomModule()(p) with HasTCacheParameters

abstract class TCacheBundle(implicit p: Parameters, val tcacheParams: TCacheParams)
    extends BoomBundle()(p) with HasTCacheParameters {
}

class TCacheMetaEntry(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends TCacheBundle()(p, tcacheParams) {
    /** The tag for the line. When invalid, the line is not present */
    val cacheTag = Valid(UInt(width = cacheTagBits.W))
    /** Does this line have changes which have not been writtenback to NLC? */
    val dirty = Bool()
}

object TCacheRequestTypeEnum extends ChiselEnum {
    /** 
     * Read a 16-byte region's tag.
     * Reads can NACK.
     */
    val READ = Value

    /** 
     * Set a 16-byte region's tag.
     * Writes can NACK. In case a write is ACK'd, all requests submitted after
     * this one (including before the ACK) will receive a correctly forwarded 
     * response. 
     */
    val WRITE = Value

    // /** Evict and writeback a line */
    // val FLUSH = Value

    /**
     * An internal request type used for handling read misses.
     * These requests cannot be sent externally, replays may only be launched
     * from the MSHRs.
     */
    val REPLAY_READ = Value

    /**
     * An internal request type used for handling write misses.
     * These requests cannot be sent externally, replays may only be launched
     * from the MSHRs.
     */
    val REPLAY_WRITE = Value

    /**
      * An internal request type used for handling writebacks.
      * Writebacks can be only launched on MSHRs, these requests cannot be 
      * These requests cannot be sent externally, writebacks may only be 
      * be launched internally.
      */
    val WRITEBACK = Value

    def is_replay(v:TCacheRequestTypeEnum.Type) = {
        v === REPLAY_READ || 
        v === REPLAY_WRITE
    }

    /**
      * Can this type of request be killed by a mispredict or exception?
      * The LSU will speculatively launch reads which may be killed but other
      * things, like writes or writebacks, are either only launched after 
      * commit or are purely internal.
      */
    def is_killable(v:TCacheRequestTypeEnum.Type) = {
        v === REPLAY_READ ||
        v === REPLAY_WRITE
    }
}

class NormalPhysicalAddress(implicit p: Parameters, tcacheParams: TCacheParams)
    extends TCacheBundle()(p, tcacheParams) {
    val address = UInt(width = coreMaxAddrBits.W)

    def getTagStoragePhysicalAddress(context:TCacheCoreIO) = {
        val tspa = Wire(Valid(new TagStoragePhysicalAddress()(p, tcacheParams)))

        // val tagIndex = address >> log2Ceil(MTEConfig.taggingGranuleBytes)
        // val byteIndex = tagIndex >> log2Ceil(8 / MTEConfig.tagBits)
        // require(MTEConfig.tagBits <= 8, "TCache assumes tags are a byte or less")
        /*
        For a 16-byte granule and 4-bit tag:
        [3 : 0] granule offset
        [4] sub-byte tag sel
        [: 5] byte region offset

        Generically:
        [granuleBytes2 - 1 : 0] granule offset
        [mteTagsPerByteBits + granuleBytes2 - 1: granuleBytes2] sub-byte tag sel
        [: 1 + mteTagsPerByteBits - 1 + granuleBytes2] byte region offset
        */
        val granuleBytes2 = log2Ceil(MTEConfig.taggingGranuleBytes)
        val subByteTagSelectIdxBegin = granuleBytes2
        val subByteTagSelectIdxEnd = log2Ceil(mteTagsPerByte) + subByteTagSelectIdxBegin - 1
        val byteRegionOffsetIdxBegin = subByteTagSelectIdxEnd + 1
        val byteRegionOffsetIdxEnd = coreMaxAddrBits - subByteTagSelectIdxEnd - 1

        tspa.bits.subByteTagSelect := address(subByteTagSelectIdxEnd, subByteTagSelectIdxBegin)
        
        val regionPack = mteRegions zip (context.mteRegionBases zip context.mteRegionMasks)
        val tagStorageAddress = Mux1H(regionPack.map {case (region, (storageBase, storageMask)) =>
            /* Generate the storage address */
            val regionByteOffset = address(byteRegionOffsetIdxEnd, byteRegionOffsetIdxBegin) - (region.base.U)(byteRegionOffsetIdxEnd, byteRegionOffsetIdxBegin)
            val storageByteAddress = storageBase + regionByteOffset

            /* Is the actual address contained by the tag region? */
            val inBounds = address >= region.base.U && address < (region.base + region.size).U && 
                /* 
                Is the storage address in bounds? 
                We check this by verifying the upper bits of the new address
                matches the upper bits of the original base (i.e. we didn't over
                flow the mask)
                */
                (storageByteAddress & ~storageMask) === (storageBase & ~storageMask) &&
                /* 
                Is the storage mask non-zero? A zero mask indicates a tagging is 
                disabled for the region 
                */
                storageMask =/= 0.U

            val v = Wire(Valid(UInt(coreMaxAddrBits.W)))
            v.valid := inBounds
            v.bits := storageByteAddress
            inBounds -> v
        })

        /* 
        We're valid if we have a valid hit. It's a SoC design error to build 
        something with overlapping regions so we're also for sure 1H
        */
        tspa.valid := tagStorageAddress.valid 
        tspa.bits.address := tagStorageAddress.bits

        tspa
    }
    
}

class TagStoragePhysicalAddress(implicit p: Parameters, tcacheParams: TCacheParams)
    extends TCacheBundle()(p, tcacheParams) {
    val address = UInt(width = coreMaxAddrBits.W)
    /** 
     * `address` refers to the byte aligned storage address. Since tags are 
     * sub-byte, we need to know which tag in the byte to select
     */  
    val subByteTagSelect = UInt(width = mteTagsPerByteBits.W)
    
    def addressOffset = address(cacheOffsetOff + cacheOffsetBits - 1, cacheOffsetOff)
    def addressIndex = address(cacheIndexOff + cacheIndexBits - 1, cacheIndexOff)
    def addressTag = address(cacheTagOff + cacheTagBits - 1, cacheTagOff)

    /** The index of this tag inside the data array line */
    def addressMTETagIndex = Cat(addressOffset, subByteTagSelect)
}

class TCacheRequest(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends TCacheBundle()(p, tcacheParams) with HasBoomUOP {
    /** The kind of request this is */
    val requestType = TCacheRequestTypeEnum()
    /** 
     * The physical address which we should act on
     * In the case of READ and WRITE operations, this is the address of the
     * memory we wish to fetch the MTE tag *for*.
     * In the case of FLUSH operations, this is the address of the backing 
     * tag storage line that needs to be flushed from this cache.
     */
    val address = UInt(width = coreMaxAddrBits.W)
    /** If this is a write request, the tag to write in, otherwise DontCare */
    val data = UInt(width = MTEConfig.tagBits.W)
}

class TCacheResponse(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends BoomBundle()(p) with HasBoomUOP {
    /** 
     * Was the request rejected? If true, all other fields are DontCare.
     * A nack'd request has no effect and should be retried later
     */
    val nack = Bool()
    /** If this is a read request, the requested tag, otherwise DontCare */
    val data = UInt(width = MTEConfig.tagBits.W)
}

class TCacheOperation(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends TCacheBundle()(p, tcacheParams) with HasBoomUOP {
    /** The kind of request this is */
    val requestType = TCacheRequestTypeEnum()
    // /** Is this request being replayed due to a miss */
    // val replay = Bool()
    /** The target, physical tag storage address this operation targets */
    val address = Valid(new TagStoragePhysicalAddress()(p, tcacheParams))
    /** 
     * The data for this operation. If this is a non-replayed write (i.e. the
     * first write), the low bits are the tag to write and the rest is DontCare.
     * If this is a replay due to the original operation missing, this is the
     * full line with the updated content (if needed, i.e. for a write).
     */
    val data = UInt(width = (tcacheParams.blockSizeBytes * 8).W)
}

// class TCacheNLCRequest(implicit p: Parameters, tcacheParams: TCacheParams) 
//     extends TCacheBundle()(p, tcacheParams) with HasBoomUOP {
//     val address = new TagStoragePhysicalAddress()(p, tcacheParams)
//     val tag_write = Valid(UInt(width = MTEConfig.tagBits.W))
// }

// class TCacheNLCResponse(implicit p: Parameters, tcacheParams: TCacheParams) 
//     extends TCacheBundle()(p, tcacheParams) with HasBoomUOP {
    
// }

class TCacheLSUIO(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends TCacheBundle()(p, tcacheParams) {
    /** For sending requests to the TCache (read/write/flush) */
    val req = Flipped(Decoupled(new TCacheRequest))
    /** For requests that require a response, this is where it comes out */
    val resp = Valid(new TCacheResponse)

    val st_exc_killed_mask = Flipped(Vec(numStqEntries, Bool()))

    /** 
     * If we're rewinding the commit queue for external (i.e. non-tcache) 
     * reasons, this is the index of the new commit head. This is necessary so
     * that the tcache can kill in-flight operations when a rewind occurs due to
     * a dmem nack, for example.
     */
    val stq_execute_head_rewind_idx = Flipped(Valid(UInt(stqAddrSz.W)))
    val stq_head_idx = Flipped(UInt(stqAddrSz.W))

    // /** 
    //  * The TCache's access port (possibly arbitrated) to the next level 
    //  * Boom DCache
    //  */
    // val nlc_req = Decoupled(new BoomDCacheReq)
    // val nlc_resp = Valid(new BoomDCacheResp)
}

class TCacheCoreIO(implicit p: Parameters) extends BoomBundle()(p) {
    val mteRegionBases = Input(Vec(mteRegions.size, UInt(width = coreMaxAddrBits.W)))
    val mteRegionMasks = Input(Vec(mteRegions.size, UInt(width = coreMaxAddrBits.W)))
    /* The tag to return for memory not contained in any tag region */
    val mtePermissiveTag = Input(UInt(width = MTEConfig.tagBits.W))
    val brupdate = Input(new BrUpdateInfo)
    val exception = Input(Bool())
    val tsc_reg     = Input(UInt())
}

class TCacheIO(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends TCacheBundle()(p, tcacheParams) {
    /** IO bundle meant to be wielded by the LSU */
    val lsu:TCacheLSUIO = new TCacheLSUIO()(p, tcacheParams)
    val core:TCacheCoreIO = new TCacheCoreIO()(p)
    val mem = Vec(tcacheParams.nMSHRs, new HellaCacheIO)
}

class BoomNonBlockingTCacheModule(
    val tcacheParams: TCacheParams
)(implicit p: Parameters)
    extends TCacheModule()(p, tcacheParams) {
    require(memWidth == 1, "TCache does not support memWidth > 1")
    require(tcacheParams.nMSHRs > 0, "nMSHRs must be non-zero")
    require(isPow2(MTEConfig.tagBits), "TCache assumes an integer number of tags fit in a byte")
    require(tcacheParams.nSets * tcacheParams.blockSizeBytes <= dcacheParams.blockBytes, "L1D blocks must land in same L0T set")
    require(tcacheParams.blockSizeBytes <= coreDataBits / 8, "BOOM dcache does not currently provide transfers larger than coreDataBits")
    val io = IO(new TCacheIO()(p, tcacheParams))
    val replacer = new RandomReplacement(tcacheParams.nWays)
    val s2_data_read_value = Wire(Vec(mteTagsPerBlock, UInt(MTEConfig.tagBits.W)))
    /* Broadcast signal to the other stages that S2 is nack-killing all writes */
    val s2_write_kill_broadcast = Wire(Bool())

    /*
    ~* Miss Status Handler Registers *~
    */
    // require(tcacheParams.nMSHRs >= 2, "nMSHRs must be >= 2 to support stall free TCache")
    val mshrsIO = Wire(Vec(tcacheParams.nMSHRs, new TCacheMSHRIO()(p, tcacheParams)))
    val mshrs = for (i <- 0 until tcacheParams.nMSHRs) yield {
        val mshr = Module(new TCacheMSHRModule(tcacheParams))
        // val dcIF = Module(new SimpleHellaCacheIF)
        // io.mem(i) <> dcIF.io.cache
        mshrsIO(i) <> mshr.io
        // dcIF.io.requestor <> mshrsIO(i).mem

        mshrsIO(i).mem <> io.mem(i)
        mshrsIO(i).exception := io.core.exception
        mshrsIO(i).brupdate := io.core.brupdate
        mshrsIO(i).tsc_reg := io.core.tsc_reg
        mshrsIO(i).st_exc_killed_mask := io.lsu.st_exc_killed_mask
        mshr
    }

    /* Select the first available MSHR, if it exists, otherwise returns the first */
    val available_mshr_idx = MuxCase(0.U, 0.until(tcacheParams.nMSHRs).map { i =>
        val mshr = mshrsIO(i)
        mshr.req_op.ready -> i.U
    })
    val available_mshr = mshrsIO(available_mshr_idx)

    /* 
    By default, leave all MSHRs in-active. Through arbitration, we'll dispatch/
    drain MSHRs as needed.
    */
    mshrsIO.foreach { mshr =>
        mshr.req_op.valid := false.B
        mshr.req_op.bits := DontCare
        mshr.resp_op.ready := false.B
        /* 
        Writebacks and other delayed data MSHR requests are submitted in S1 to 
        reserve the MSHR but their data arrives in S2. Provide the MSHR
        access to this delayed data.
        */
        mshr.s1_data := s2_data_read_value.asUInt
    }

    /*
    ~* Main IO interface *~
    */
    /* 
    The tcache pipeline provides back pressure on the LSU when it cannot find an
    MSHR to handle a miss.
    */
    val s0_is_executing_replay = Wire(Bool())
    io.lsu.req.ready := !s0_is_executing_replay

    when (io.lsu.req.fire) {
        printf("[tcache] accepted request for addr=0x%x, op=%d [tsc=%d]\n", io.lsu.req.bits.address, io.lsu.req.bits.requestType.asUInt, io.core.tsc_reg)
    }

    /*
    Since this is a fairly small cache with small lines, we don't really want
    to create multiple data way SRAMs due to the overhead per SRAM. At the same
    time though, we don't really want to have an SRAM read width 128bits for
    2-way or 256 for 4-way. Worse, the tag mux would end up being 4x64 which is
    not great. Instead we could try to split the way elements into rows of data 
    array. This, however, prevents us from launching a meta and data read at the 
    same time for read operations, which is somewhat annoying. We can work
    around this through sketchier pipelining.

    Call it area and power efficient cache design, or something lol.

    S0:
        * New request comes in combinatorially
        * Arbitrate request
        * Setup meta read
    S1: 
        * Check for meta hit
        * If read, launch read for hit. If we miss, try to launch fill.
        * If write hit, launch metadata write
          If write miss, try to launch a fill (i.e. DO NOT write anything on miss)
        * If replay, launch metadata write. If the evicted line is dirty, launch a
          a data read for the victim and a writeback request to the MSHR. The data
          for the writeback will not arrive until S2 but we need to reserve the
          MSHR since replays cannot NACK. 
        
        A replay should never nack since  while it is in S0, a miss in S1
        cannot use the replay's MSHR as it is still blocked. When the replay
        enters S1, the MSHR will be free again to handle a possible writeback.
        This means we can fearlessly fire fills without needing to worry about
        stalling.

        TODO: Forward metadata writes backwards in-case the address in S0 is 
        the same as we're writing here.
    S2:
        * Mux out the tag result. The tag data will probably arrive late but
          it doesn't really matter since clients will probably be stuffing it in
          a register anyways (i.e. the LDQ/STQ)
        * If write, launch masked data write
        * If replay, launch unmasked data write

        TODO: Forward data writes backwards in-case the address in S1 is the same
        as we're writing here
    */
    val dataArraysEntries = tcacheParams.nSets * tcacheParams.nWays
    val dataArrays = DescribedSRAM(
        name = s"tcache_data",
        desc = "Non-blocking TCache Data Array",
        size = dataArraysEntries,
        /* One block per row */
        data = Vec(mteTagsPerBlock, Bits(MTEConfig.tagBits.W))
    )

    val metaT = new TCacheMetaEntry()(p, tcacheParams)
    val metaWidth = metaT.getWidth
    val metaArrays = DescribedSRAM(
        name = s"tcache_meta",
        desc = "Non-blocking TCache Metadata Array",
        size = tcacheParams.nSets,
        data = Vec(tcacheParams.nWays, Bits(metaWidth.W))
    )

    def op_is_write_killed(op:TCacheOperation):Bool = {
        /* If S2 is nacking a write, all younger writes need to be killed */
        s2_write_kill_broadcast && op.requestType === TCacheRequestTypeEnum.WRITE
    }
    def op_is_still_valid(op:TCacheOperation):Bool = {
        !IsKilledByBranch(io.core.brupdate, op.uop) &&
        !(io.core.exception && op.uop.uses_ldq) &&
        !(op.uop.uses_stq && io.lsu.st_exc_killed_mask(op.uop.stq_idx))
    }

    /* ~* Stage 0 *~ */

    /* Decode the incoming request */
    val s0_incoming_req = io.lsu.req
    val s0_incoming_op = Wire(new TCacheOperation()(p, tcacheParams))
    val s0_incoming_op_valid = s0_incoming_req.valid
    s0_incoming_op.requestType := s0_incoming_req.bits.requestType
    s0_incoming_op.data := Cat(Fill(tcacheParams.blockSizeBytes - MTEConfig.tagBits, 0.U), s0_incoming_req.bits.data)
    s0_incoming_op.uop := s0_incoming_req.bits.uop
    assert(!s0_incoming_op_valid || !TCacheRequestTypeEnum.is_replay(s0_incoming_op.requestType), "Replays may not be launched externally")

    val s0_normal_address = Wire(new NormalPhysicalAddress()(p, tcacheParams))
    s0_normal_address.address := s0_incoming_req.bits.address
    s0_incoming_op.address := s0_normal_address.getTagStoragePhysicalAddress(io.core)


    /*
    ** Request arbitration **
    Since the TCache can, in theory, be inundated with an infinite stream of 
    hitting requests, we must prioritize draining the MSHRs over incoming
    requests in order to prevent the missed requests from starving. Since we 
    always prioritize draining MSHRs over incoming requests and because a replay
    can never miss, we don't have the case where we continuously drain one MSHR
    while leaving the rest to starve.
    */
    val s0_op = Wire(new TCacheOperation()(p, tcacheParams))
    val s0_op_valid = Wire(Bool())

    val completed_mshr_idx = MuxCase(0.U, 0.until(tcacheParams.nMSHRs).map { i =>
        val mshr = mshrsIO(i)
        mshr.resp_op.valid -> i.U
    })
    val completed_mshr = mshrsIO(completed_mshr_idx)
    /* When this is true, we bring req.ready low to reject the incoming request */
    s0_is_executing_replay := completed_mshr.resp_op.valid

    when (s0_is_executing_replay) {
        completed_mshr.resp_op.ready := true.B
        s0_op := completed_mshr.resp_op.bits
        s0_op_valid := true.B
        printf("[tcache] executing replay [tsc=%d]\n", io.core.tsc_reg)
    } .otherwise {
        s0_op := s0_incoming_op
        s0_op_valid := s0_incoming_op_valid
    }

    when (s0_op_valid) {
        assert(s0_is_executing_replay ^ io.lsu.req.fire, "Arbiter drew from two sources?")
        printf("[tcache] decoded tag storage address -> %x (sel=%x) uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d. type=%d, data=%x [tsc=%d]\n", s0_op.address.bits.address, s0_op.address.bits.subByteTagSelect, s0_op.uop.uses_ldq, s0_op.uop.ldq_idx, s0_op.uop.uses_stq, s0_op.uop.stq_idx, s0_op.requestType.asUInt, s0_op.data, io.core.tsc_reg)
    }

    /* ~* Stage 1 *~ */
    val s1_op = RegNext(s0_op)
    s1_op.uop.br_mask := GetNewBrMask(io.core.brupdate, s0_op.uop)
    val s1_op_valid = RegNext(
        s0_op_valid && op_is_still_valid(s0_op) && !op_is_write_killed(s0_op)
    )
        
    val s1_meta_read_value = Wire(Vec(tcacheParams.nWays, metaT))
    /* 
    forward declared wires for hazard detection. We need these otherwise we get
    funky null pointer exceptions on elaboration
    */
    val s1_s2_op = Wire(new TCacheOperation()(p, tcacheParams))
    val s1_s2_op_valid = Wire(Bool())
    val s1_s2_nack = Wire(Bool())

    /* We always need to access meta but don't always care to read data */
    s1_meta_read_value := metaArrays.read(
        s0_op.address.bits.addressIndex, 
        s0_op_valid && s0_op.address.valid
    ).map{ _.asTypeOf(metaT) }

    /* 
    If a request for this same line is already missed out, we want to block the
    request and nack it instead
    */
    val s1_mshr_blocked = mshrsIO.map { mshrIO =>
        val align_mask = (tcacheParams.blockSizeBytes - 1).U(coreMaxAddrBits.W)
        mshrIO.blocked_address.valid && 
        mshrIO.blocked_address.bits === (s1_op.address.bits.address & ~align_mask)}.reduce(_ || _)

    /* Hit all the ways */
    val s1_way_hit_map = VecInit(s1_meta_read_value.map {way_meta =>
        way_meta.cacheTag.valid && 
            way_meta.cacheTag.bits === s1_op.address.bits.addressTag
    })
    val s1_way_hit_idx = Mux1H(
        (s1_way_hit_map zip 0.until(tcacheParams.nWays)).map {case (hit, idx) =>
            hit -> idx.U(nWaysBits.W)
        }
    )
    val s1_hit = s1_way_hit_map.reduce(_ || _)

    assert(!s1_op_valid || (s1_hit && !s1_mshr_blocked || !s1_hit), "Hit in cache but there is an outstanding MSHR for the same line...")

    var s1_debug_has_hit = false.B
    s1_way_hit_map.foreach { hit =>
        assert(!s1_debug_has_hit || s1_debug_has_hit && !hit, "Multiple ways in TCache hit?")
        s1_debug_has_hit = hit
    }

    /* 
    Since we should only launch misses for missing lines and we only should have
    one active miss per line (enforced by NACKing any requests which land on an
    actively missed out line), we should never have a case where a replay finds
    its data is already in the cache
    */
    assert(!s1_op_valid || (!TCacheRequestTypeEnum.is_replay(s1_op.requestType) || !s1_hit), "Attempting to execute replay when line is not missing")

    val s1_needs_miss = s1_op_valid && s1_op.address.valid && !s1_hit && 
                        (s1_op.requestType === TCacheRequestTypeEnum.READ || 
                         s1_op.requestType === TCacheRequestTypeEnum.WRITE) 
    
    /*
    Since the read result of simultaneously writing and reading the same address
    on an array is undefined, we nack any operations which will have either
    generated a hazard. The only op that could pose a hazard to us is the one
    currently in S2 because when we were in S0 it was writing meta in S1 (hazard)
    and now that we're in S1 it is writing the data we want to read in S2.
    */
    val s1_hazard_blocked = Wire(Bool())

    /* We only have a hazard if they targeted the same record as us */
    val s1_idx_matches_s2 = s1_s2_op.address.bits.addressIndex === s1_op.address.bits.addressIndex
    when (!s1_s2_op_valid) {
        /* Trivially hazard free -- nothing should be happening here */
        s1_hazard_blocked := false.B
    } .elsewhen (s1_s2_op.requestType === TCacheRequestTypeEnum.WRITE) {
        /*
        TODO: We might actually be able to get craftier and allow back to back
        writes so long as we're sure metadata won't be corrupted.
        */
        /* The hazard only exists if the write actually happened */
        s1_hazard_blocked := !s1_s2_nack && s1_idx_matches_s2
    } .elsewhen (TCacheRequestTypeEnum.is_replay(s1_s2_op.requestType)) {
        /* 
        If the previous operation was a replay, our entire metadata is undefined
        and so there's not much we can do.
        */ 
        s1_hazard_blocked := s1_idx_matches_s2
    } .otherwise {
        /* 
        All other request types either don't flow down this pipeline or do not
        write
        */
        s1_hazard_blocked := false.B
    }
    val s1_nack = 
        (s1_needs_miss && !available_mshr.req_op.ready) || 
        s1_mshr_blocked ||
        s1_hazard_blocked
    
    // The index to read to get the data for the hit way
    val s1_hit_data_array_idx = 
        Cat(s1_op.address.bits.addressIndex, s1_way_hit_idx)

    /* Handle metadata write */
    val s1_meta_wmask = Wire(Vec(tcacheParams.nWays, Bool()))
    val s1_meta_wdata = Wire(Vec(tcacheParams.nWays, UInt(metaT.getWidth.W)))
    
    // The index of our victim if we evicted someone.
    val s1_victim_way_idx = Wire(Valid(UInt(log2Ceil(tcacheParams.nWays).W)))
    s1_victim_way_idx.valid := false.B
    s1_victim_way_idx.bits := DontCare

    when (s1_op_valid && !s1_nack &&
          s1_op.requestType === TCacheRequestTypeEnum.WRITE && s1_hit) {
        // assert(!s1_nack, "A nack'd operation is attempting to write metadata")
        /* Write hit. We simply need to mark the line as dirty */
        // Write to the way we hit
        s1_meta_wmask := s1_way_hit_map
        
        // Update the hit meta entry to reflect that it was dirtied
        val new_meta = Wire(metaT)
        new_meta := s1_meta_read_value(s1_way_hit_idx)
        new_meta.dirty := true.B
        s1_meta_wdata := VecInit(Seq.fill(tcacheParams.nWays)(new_meta.asUInt))
    }.elsewhen (s1_op_valid && !s1_nack &&
                TCacheRequestTypeEnum.is_replay(s1_op.requestType)) {
        // assert(!s1_nack, "A nack'd operation is attempting to write metadata")
        /*
        Replays are generated for misses and so we need to pick a victim and
        evict them in order to generate our write.
        */
        replacer.miss
        s1_victim_way_idx.valid := true.B
        s1_victim_way_idx.bits := replacer.way

        s1_meta_wmask := UIntToOH(
            s1_victim_way_idx.bits, 
            tcacheParams.nWays
        ).asBools

        /* Insert the new meta entry */
        val new_meta = Wire(metaT)
        new_meta.cacheTag.valid := true.B
        new_meta.cacheTag.bits  := s1_op.address.bits.addressTag
        /* 
        The entry must be inserted dirty if this is a write replay since the
        data we're inserting here actually includes the updated data already
        */
        new_meta.dirty := s1_op.requestType === TCacheRequestTypeEnum.REPLAY_WRITE
        s1_meta_wdata := VecInit(Seq.fill(tcacheParams.nWays)(new_meta.asUInt))
    }.otherwise {
        /* We're not writing to meta, mask off the write */
        s1_meta_wmask := VecInit(Seq.fill(tcacheParams.nWays)(false.B))
        s1_meta_wdata := DontCare
    }

    metaArrays.write(
        s1_op.address.bits.addressIndex,
        s1_meta_wdata,
        s1_meta_wmask
    )
    dontTouch(s1_op.address.bits.addressIndex)
    dontTouch(s1_meta_wdata)
    dontTouch(s1_meta_wmask)

    /* Handle writebacks */
    val s1_replay_writeback_meta_e = s1_meta_read_value(s1_victim_way_idx.bits)
    val s1_replay_needs_writeback = 
        s1_op_valid &&
        TCacheRequestTypeEnum.is_replay(s1_op.requestType) &&
        s1_victim_way_idx.valid &&
        s1_replay_writeback_meta_e.cacheTag.valid &&
        s1_replay_writeback_meta_e.dirty

    val s1_replay_writeback_data_array_idx = 
        Cat(s1_op.address.bits.addressIndex, s1_victim_way_idx.bits)

    /* Drive the data array read port for S2 */
    val s1_data_array_enable = Wire(Bool())
    val s1_data_array_idx = Wire(UInt(log2Ceil(dataArraysEntries).W))
    s1_data_array_enable := false.B
    s1_data_array_idx := DontCare
    when (s1_op_valid) {
        when (s1_replay_needs_writeback) {
            s1_data_array_enable := true.B
            s1_data_array_idx := s1_replay_writeback_data_array_idx
        } .elsewhen(s1_op.requestType === TCacheRequestTypeEnum.READ &&
                    s1_op.address.valid && s1_hit) {
            s1_data_array_enable := true.B
            s1_data_array_idx := s1_hit_data_array_idx
        } 
    }

    /* Handle MSHR dispatch */
    when (s1_replay_needs_writeback) {
        /*
        Submit a writeback request to the MSHR
        This should never fail, as discussed early in the pipeline description, 
        since replays are pulled directly from an MSHR (thereby freeing an MSHR
        in the cycle), which means the replay in S1 gets first dibs.
        */
        assert(available_mshr.req_op.ready, "An MSHR is expected to always be free when handling a replay writeback??")
        available_mshr.req_op.valid := true.B

        /*
        To writeback a line, we need to synthesize a request from the data
        we want to writeback rather than the data we were replaying for in s2_op
        */
        val req_b = available_mshr.req_op.bits
        req_b.requestType := TCacheRequestTypeEnum.WRITEBACK
        req_b.uop := NullMicroOp

        assert(s1_replay_writeback_meta_e.cacheTag.valid, "Attempting to writeback an invalid line?")
        req_b.address.valid := true.B
        req_b.address.bits.address := Cat(
            /* just add water: reconstitute the address from tag and index */
            s1_replay_writeback_meta_e.cacheTag.bits,
            s1_op.address.bits.addressIndex << cacheIndexOff
        )
        req_b.address.bits.subByteTagSelect := 0.U

        /*
        We don't have the data for this request yet but we're launching it this
        cycle. Writeback requests, however, use the delayed data port `s1_data`
        to get the data, so we're fine.
        */
        req_b.data := DontCare
        printf("[tcache] s1_writeback address=%x, way=%d. Evicted because address=%x [tsc=%d]\n", req_b.address.bits.address, s1_victim_way_idx.bits, s1_op.address.bits.address, io.core.tsc_reg)

    } .elsewhen (s1_needs_miss && !s1_nack) {
        assert(available_mshr.req_op.ready, "A request was not nack'd even though no MSHR is available?")
        /* 
        Simple misses can be directly passed off to the MSHR. When they
        complete, a new replay operation will be submitted.
        */
        val uop = s1_op.uop
        available_mshr.req_op.valid := 
            s1_op_valid && op_is_still_valid(s1_op) && !op_is_write_killed(s1_op)
        available_mshr.req_op.bits := s1_op
        available_mshr.req_op.bits.uop.br_mask := GetNewBrMask(io.core.brupdate, uop)
    }

    /* ~* Stage 2 *~ */

    val s2_op = RegNext(s1_op)
    s2_op.uop.br_mask := GetNewBrMask(io.core.brupdate, s1_op.uop)

    val s2_op_valid = RegNext(
        s1_op_valid && 
        op_is_still_valid(s1_op) && !op_is_write_killed(s1_op)
    )
    val s2_op_type = s2_op.requestType
    val s2_hit = RegNext(s1_hit)
    val s2_nack = RegNext(s1_nack)
    val s2_way_hit_idx = RegNext(s1_way_hit_idx)
    val s2_victim_way_idx = RegNext(s1_victim_way_idx)
    val s2_debug_needs_miss = RegNext(s1_needs_miss)

    s1_s2_op_valid := s2_op_valid
    s1_s2_op := s2_op
    s1_s2_nack := s2_nack

    s2_data_read_value := dataArrays.read(
        s1_data_array_idx, 
        s1_data_array_enable
    )

    /*
    Contract: when a write nacks, kill all younger writes in the pipeline with a
    nack. We do this in S2 so that clients don't need to worry about 
    combinatorially killing their requests when they get a nack response here.
    */
    s2_write_kill_broadcast := s2_op_valid && 
        s2_nack &&
        s2_op.requestType === TCacheRequestTypeEnum.WRITE

    /* Craft the response */
    val resp = io.lsu.resp
    val resp_v = resp.valid
    val resp_b = resp.bits
    resp_b.uop := s2_op.uop

    when (!s2_op_valid) {
        /* The request isn't valid, so we can't possibly have anything to say */
        resp_v := false.B
        resp_b := DontCare
    } .elsewhen(s2_op_type === TCacheRequestTypeEnum.READ) {
        when (!s2_op.address.valid) {
            /* 
            If the request is good but the address is not, this indicates that
            we launched a request for a region not covered by MTE. In this case,
            we simply return the permissive tag
            */
	        printf("[tache] s2 resp default = uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d [tsc=%d]\n", s2_op.uop.uses_ldq, s2_op.uop.ldq_idx, s2_op.uop.uses_stq, s2_op.uop.stq_idx, io.core.tsc_reg) 
            resp_v := true.B
            resp_b.data := io.core.mtePermissiveTag
            resp_b.nack := false.B
        } .otherwise {
            /*
            We had a valid address, so try to return the hit. If we missed,
            we already tried to pushed to the MSHR in S1 and will nack if we
            failed.
            */
            assert(s2_hit || s2_debug_needs_miss, "S2 read miss but no miss was fired in S1")
            printf("[tcache] s2 read hit=%d, nack=%d, addr=%x, uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d [tsc=%d]\n", s2_hit, s2_nack, s2_op.address.bits.address, s2_op.uop.uses_ldq, s2_op.uop.ldq_idx, s2_op.uop.uses_stq, s2_op.uop.stq_idx, io.core.tsc_reg)
            resp_v := s2_hit || s2_nack
            resp_b.nack := s2_nack 

            resp_b.data := s2_data_read_value(s2_op.address.bits.addressMTETagIndex)
        }
    } .elsewhen (s2_op_type === TCacheRequestTypeEnum.REPLAY_READ) {
        /*
        Read replay response
        We want to serve the response data out of the replay packet rather than
        out of the data array since we may be using the array to read the 
        writeback data.
        Replays cannot NACK, so these responses are always valid.
        */
        printf("[tcache] s2 replay read addr=%x, uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d [tsc=%d]\n", s2_op.address.bits.address, s2_op.uop.uses_ldq, s2_op.uop.ldq_idx, s2_op.uop.uses_stq, s2_op.uop.stq_idx, io.core.tsc_reg)
        
        resp_v := true.B
        resp_b.nack := false.B

        /* Reshape the packet data into a cache data line */
        val data = Wire(Vec(mteTagsPerBlock, UInt(MTEConfig.tagBits.W)))
        data := s2_op.data.asTypeOf(data)

        resp_b.data := data(s2_op.address.bits.addressMTETagIndex)
    } .elsewhen (s2_op_type === TCacheRequestTypeEnum.WRITE) {
        /*
        Write response
        Writes are special in that we can early ACK them even if we missed
        because we can guarantee that the write replay can't fail. Thus, we 
        always respond here. Since we NACK all requests which have an active
        miss and we forward in the pipeline, nobody can really tell the
        difference between a true write hit and a write miss which we were
        able to miss out.
        Early acks are helpful because it saves clients from needing to wait for
        a possibly very long miss.
        */
        printf("[tcache] s2 write ack addr=%x, uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d (nack=%d, hit=%d, data=%x) [tsc=%d]\n",
            s2_op.address.bits.address, s2_op.uop.uses_ldq, s2_op.uop.ldq_idx, s2_op.uop.uses_stq, s2_op.uop.stq_idx,
            s2_nack, s2_hit,
            s2_op.data,
            io.core.tsc_reg
        )
        resp_v := true.B
        resp_b.nack := s2_nack
        resp_b.data := DontCare

    } .elsewhen (s2_op_type === TCacheRequestTypeEnum.REPLAY_WRITE) {
        /*
        Write replay response
        Writes are early ack'd and so write replays don't need to respond.
        */
        printf("[tcache] s2 replay write addr=%x, uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d (hit=%d, data=%x) [tsc=%d]\n",
            s2_op.address.bits.address, s2_op.uop.uses_ldq, s2_op.uop.ldq_idx, s2_op.uop.uses_stq, s2_op.uop.stq_idx,
            s2_hit,
            s2_op.data,
            io.core.tsc_reg
        )
        resp_v := false.B
        resp_b.nack := DontCare
        resp_b.data := DontCare
    } .otherwise {
    	assert(false.B, "TCache unexpected s2_op type")
    }

    /* Handle data writes */
    // The victim way takes priority over the hit since when we have a victim
    // we are always going to be writing to that way (replay)
    assert(!s2_victim_way_idx.valid || s2_victim_way_idx.valid && TCacheRequestTypeEnum.is_replay(s2_op_type), "Victim for non-replay?")
    val s2_data_wway_idx = Mux(s2_victim_way_idx.valid, s2_victim_way_idx.bits, s2_way_hit_idx)
    val s2_data_widx = Cat(s2_op.address.bits.addressIndex, s2_data_wway_idx)
    val s2_data_wmask = Wire(Vec(mteTagsPerBlock, Bool()))
    val s2_data_wdata = Wire(dataArrays.t)
    // Reshape the request to be a vector like the array
    val s2_op_data = Wire(Vec(mteTagsPerBlock, UInt(MTEConfig.tagBits.W)))
    s2_op_data := s2_op.data.asTypeOf(s2_op_data)
    when(s2_op_valid &&
               s2_op_type === TCacheRequestTypeEnum.WRITE && s2_hit) {
        /* Mask the write so we only hit the appropriate tag */
        val offset = Cat(s2_op.address.bits.address(s2_op.cacheOffsetBits, 0), 
                         s2_op.address.bits.subByteTagSelect)
        s2_data_wmask := UIntToOH(offset, mteTagsPerBlock).asBools
        s2_data_wdata := VecInit(Seq.fill(mteTagsPerBlock){
            s2_op.data(MTEConfig.tagBits - 1, 0)
        })
                              
    } .elsewhen(s2_op_valid &&
                TCacheRequestTypeEnum.is_replay(s2_op_type)) {
        /* Replays are line fills, so dump the entire data argument in */
        s2_data_wmask := VecInit(Seq.fill(mteTagsPerBlock)(true.B))
        s2_data_wdata := s2_op_data
    } .otherwise {
        s2_data_wmask := VecInit(Seq.fill(mteTagsPerBlock)(false.B))
        s2_data_wdata := DontCare
    }

    dontTouch(s2_data_widx)
    dontTouch(s2_data_wdata)
    dontTouch(s2_data_wmask)
    dataArrays.write(
        s2_data_widx,
        s2_data_wdata,
        s2_data_wmask
    )
}

class TCacheMSHRIO(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends TCacheBundle()(p, tcacheParams) {
    /** 
     * The operation that missed. This operation will be fulfilled and passed
     * back on resp_op to be replayed
     */
    val req_op = Flipped(DecoupledIO(new TCacheOperation))
    /** When valid, the completed miss ready to be directly replayed. */
    val resp_op = DecoupledIO(new TCacheOperation)
    /** 
     * For request types that take delayed data (i.e. WRITEBACK), this port
     * provides that data
     */
    val s1_data = Input(UInt((tcacheParams.blockSizeBytes * 8).W))

    /** This misshandler's request to and response from NLC */
    val mem = new HellaCacheIO

    val brupdate = Input(new BrUpdateInfo)
    val exception = Input(Bool())
    val st_exc_killed_mask = Flipped(Vec(numStqEntries, Bool()))
    val tsc_reg = Input(UInt())

    val blocked_address = Valid(UInt(coreMaxAddrBits.W))
}

object TCacheMSHRState extends ChiselEnum {
    /** Ready to accept a new request */
    val READY = Value
    /** Waiting for downstream memory be ready to accept a request */
    val MEMORY_REQUEST_WAIT = Value
    /** Waiting for downstream memory to respond to an issued request */
    val MEMORY_RESPONSE_WAIT = Value
    /** Request is completed, waiting for TCache to drain the MSHR */
    val TCACHE_WAIT = Value
}

// class TCacheMSHRModule2(
//     val tcacheParams: TCacheParams
// )(implicit p: Parameters) extends TCacheModule()(p, tcacheParams)
//     with freechips.rocketchip.rocket.constants.MemoryOpConstants {
//     val io = IO(new TCacheMSHRIO()(p, tcacheParams))
    
//     val next_state = Wire(TCacheMSHRState())
//     val state = RegInit(TCacheMSHRState.READY)
//     state := next_state

//     val op_r = RegInit({
//         val op_i = Wire(Valid(new TCacheOperation()(p, tcacheParams)))
//         op_i.valid := false.B
//         op_i.bits := DontCare
//         op_i
//     })
//     val op = Wire(Valid(new TCacheOperation))
//     // val op = Mux(state === TCacheMSHRState.READY, io.req_op.bits, op_r.bits)
//     // val op_v = Wire(Bool())
//     val op_v = !(state === TCacheMSHRState.READY && next_state === TCacheMSHRState.READY) &&
//         !IsKilledByBranch(io.brupdate, op.uop) &&
//         !(io.exception && op.requestType === TCacheRequestTypeEnum.READ)
//     dontTouch(io.req_op.bits.uop)

//     val s1_data_r = Reg(Valid(UInt((tcacheParams.blockSizeBytes * 8).W)))
//     /* 
//     We only serve from the latched data if we had to stall the write and thus
//     had time to latch it
//     */
//     val s1_data = Mux(s1_data_r.valid, s1_data_r.bits, io.s1_data)

//     val blocked_address_r = Reg(UInt(coreMaxAddrBits.W))
//     io.blocked_address.bits := blocked_address_r
//     /* 
//     The request that we're blocked on is only valid when processing a request 
//     */
//     io.blocked_address.valid := (state === TCacheMSHRState.READY)

//     /* Generate the next state */
//     when (state === TCacheMSHRState.READY) {
//         when (io.req_op.fire) {
//             /* New request coming in! */
//             when (io.mem.req.fire) {
//                 /* We fired comb., wait for response */
//                 next_state := TCacheMSHRState.MEMORY_RESPONSE_WAIT
//             } .otherwise {
//                 /* We didn't manage to fire to memory, try again next cycle */
//                 next_state := TCacheMSHRState.MEMORY_REQUEST_WAIT
//             }
//         } .otherwise {
//             /* No new request, so we're still ready */
//             next_state := TCacheMSHRState.READY
//         }
//     } .elsewhen(state === TCacheMSHRState.MEMORY_REQUEST_WAIT) {
//         /* This state retries firing to memory */
//         when (io.mem.req.fire) {
//             /* We fired, wait for response */
//             next_state := TCacheMSHRState.MEMORY_RESPONSE_WAIT
//         } .otherwise {
//             /* We didn't manage to fire to memory, try again next cycle */
//             next_state := TCacheMSHRState.MEMORY_REQUEST_WAIT
//         }
//     } .elsewhen (state === TCacheMSHRState.MEMORY_RESPONSE_WAIT) {
//         when (io.mem.resp.valid) {
//             when (io.resp_op.fire) {
//                 /* 
//                 We short circuit returned the response to the TCache, ready to
//                 accept a new request
//                 */
//             next_state := TCacheMSHRState.READY
//             } .otherwise {
//                 /* TCache wasn't ready for us, go to wait */
//                 next_state := TCacheMSHRState.TCACHE_WAIT
//             }
//         } .otherwise {
//             /* Nothing came :( Wait forlornly by the mailbox */
//             next_state := TCacheMSHRState.MEMORY_RESPONSE_WAIT
//         }
//     } .otherwise {
//         next_state := DontCare
//         assert(false.B, "state is impossible")
//     }

//     /* Manage the internal operation fastpaths */
//     when (state === TCacheMSHRState.READY) {
//        when (next_state =/= )
//     }

// }
class TCacheMSHRModule(
    val tcacheParams: TCacheParams
)(implicit p: Parameters) extends TCacheModule()(p, tcacheParams)
    with freechips.rocketchip.rocket.constants.MemoryOpConstants {
    val io = IO(new TCacheMSHRIO()(p, tcacheParams))
    
    val op_r = RegInit({
        val op_i = Wire(Valid(new TCacheOperation()(p, tcacheParams)))
        op_i.valid := false.B
        op_i.bits := DontCare
        op_i
    })
    dontTouch(io.req_op.bits.uop)

    val blocked_address_r = Reg(UInt(coreMaxAddrBits.W))
    io.blocked_address.bits := blocked_address_r

    /** Have we fired the request into memory? */
    val executed_r = Reg(Bool())
    /** Have we fired a request into memory and are waiting for it to return? */
    val memory_executing = RegInit(false.B)

    val s1_data_r = Reg(Valid(UInt((tcacheParams.blockSizeBytes * 8).W)))

    /* 
    We can accept a new request when we're done with the one we have and no we
    have no pending responses from memory (i.e. we're fully clean)
    */
    io.req_op.ready := !op_r.valid && !memory_executing
    when (io.req_op.fire) {
    	assert(io.req_op.valid && io.req_op.bits.address.valid, "MSHR was fired an invalid op?")
        assert(!TCacheRequestTypeEnum.is_replay(io.req_op.bits.requestType), "MSHR was fired a replay?")
        printf("[mshr] accepted new op, addr=%x, uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d [tsc=%d]\n", io.req_op.bits.address.bits.address, io.req_op.bits.uop.uses_ldq, io.req_op.bits.uop.ldq_idx, io.req_op.bits.uop.uses_stq, io.req_op.bits.uop.stq_idx, io.tsc_reg)

        op_r.bits := io.req_op.bits
        executed_r := false.B
        s1_data_r.valid := false.B
        s1_data_r.bits := DontCare
    }

    /* Latch S1 data in S1 */
    /* 
    Fast path access to s1_data 
    We can fast as soon as S1 but we want to deactivate the fast path after S1.
    op_r.valid is a good analogue as it is invalid in S0 but valid for all
    cycles after (so op_r.valid is equiv to "is S1 or later").
    s1_data.valid is equiv to "is S2 or later" as it is set in S1.
    */
    val s1_data = Mux(s1_data_r.valid, s1_data_r.bits, io.s1_data)
    when (!s1_data_r.valid) {
        s1_data_r.valid := true.B
        s1_data_r.bits := io.s1_data
    }
    dontTouch(s1_data_r)
    
    val op = Mux(io.req_op.fire, io.req_op.bits, op_r.bits)
    val op_address = op.address
    val op_type = op.requestType
    val op_v = Mux(io.req_op.fire, io.req_op.valid, op_r.valid) &&
        !IsKilledByBranch(io.brupdate, op.uop) &&
        !(io.exception && op.uop.uses_ldq) &&
        !(op.uop.uses_stq && io.st_exc_killed_mask(op.uop.stq_idx))
    val executed = Mux(io.req_op.fire, false.B, executed_r && !io.mem.s2_nack)
    
    op_r.valid := op_v &&
                    /* Invalidate after we fire the response */
                    !io.resp_op.fire &&
                    /* Invalidate writeback requests on write ACK */
                    (op.requestType =/= TCacheRequestTypeEnum.WRITEBACK ||
                            !io.mem.resp.valid)
    op_r.bits.uop.br_mask := GetNewBrMask(io.brupdate, op.uop)
    io.blocked_address.valid := op_r.valid


    /* ~* Launch the memory request *~ */
    val req_b = io.mem.req.bits
    val req_v = io.mem.req.valid

    io.mem.s1_kill := false.B
    io.mem.s2_kill := false.B
    io.mem.req.bits.data := 0xaaadead.U
    io.mem.s1_data.data := s1_data
    io.mem.s1_data.mask := Fill(coreDataBytes, 1.U(1.W))
    req_v := false.B
    req_b := DontCare

    /* Tag regions are configured with physical addresses */
    req_b.phys := true.B
    req_b.size := log2Ceil(tcacheParams.blockSizeBytes).U
    req_b.signed := false.B
    req_b.dv := false.B /* ??? */
    req_b.dprv := PRV.S.U
    req_b.no_alloc := false.B
    req_b.no_xcpt := false.B
    val align_mask = (tcacheParams.blockSizeBytes - 1).U(coreMaxAddrBits.W)
    val masked_addr = op_address.bits.address & ~align_mask
    req_b.addr := masked_addr
    blocked_address_r := masked_addr

    /* Try to fire memory requests and retries */
    when (op_v && !executed) {
        when (op_type === TCacheRequestTypeEnum.READ ||
              op_type === TCacheRequestTypeEnum.WRITE) {
            /* Line miss, read data from NLC */
            req_v := true.B
            req_b.cmd := M_XRD
        } .elsewhen(op_type === TCacheRequestTypeEnum.WRITEBACK) {
            /* Line writeback. Data flows through s1_data */
            req_v := true.B
            req_b.cmd := M_XWR
        } .otherwise {
            assert(false.B, "not implemented")
        }
    }

    when (io.mem.s2_nack) {
        printf("[mshr] s2_nack req, retrying addr=%x, cmd=%x, alive=%d, [tsc=%d]\n", io.mem.req.bits.addr, io.mem.req.bits.cmd, op_v, io.tsc_reg)
        // Set executed to false on nack so we retry
        // If we retry this cycle (executed includes nack), the next block
        // will override this write and make it true.
        executed_r := false.B
        memory_executing := false.B
    }

    when (op_v && io.mem.req.fire) {
        printf("[mshr] firing req addr=%x, cmd=%x [tsc=%d]\n", io.mem.req.bits.addr, io.mem.req.bits.cmd, io.tsc_reg)
        executed_r := true.B
        memory_executing := true.B
    }

    /* ~* Handle memory response *~ */
    val resp_op_v_r = Reg(Bool())
    val resp_mem_v = io.mem.resp.valid
    val should_send_resp = op_r.valid && 
        op_r.bits.requestType =/= TCacheRequestTypeEnum.WRITEBACK &&
        (resp_mem_v || resp_op_v_r) &&
        !IsKilledByBranch(io.brupdate, op_r.bits.uop) &&
        !(io.exception && op_r.bits.uop.uses_ldq) &&
        !(op_r.bits.uop.uses_stq && io.st_exc_killed_mask(op_r.bits.uop.stq_idx))
        
    val resp_mem_b = io.mem.resp.bits

    val resp_line_r = Reg(UInt(tcacheParams.blockSizeBytes.W))
    val resp_line = Mux(resp_mem_v, resp_mem_b.data, resp_line_r)

    val resp_op_b = io.resp_op.bits
    // We use op_r here rather than op_v to break the combinational loop
    // we don't need op_v anyways since we'll never be valid in cycle 0
    io.resp_op.valid := should_send_resp
    resp_op_b := op_r.bits
    resp_op_b.uop.br_mask := GetNewBrMask(io.brupdate, op_r.bits.uop)
    when (op_r.bits.requestType === TCacheRequestTypeEnum.READ) {
        resp_op_b.requestType := TCacheRequestTypeEnum.REPLAY_READ
    } .elsewhen (op_r.bits.requestType === TCacheRequestTypeEnum.WRITE) {
        resp_op_b.requestType := TCacheRequestTypeEnum.REPLAY_WRITE
    } .elsewhen(op_r.bits.requestType === TCacheRequestTypeEnum.WRITEBACK) {
        resp_op_b.requestType := TCacheRequestTypeEnum.WRITEBACK
    } .otherwise {
        assert(false.B, "Unexpected request type in MSHR")
        resp_op_b.requestType := DontCare
    }

    /* 
    Process the write request, if needed.
    We process the write here rather than somewhere sensible like in S2 of the
    TCache as a hack derived from the fact that resp.data is used for both for
    single tags (in a WRITE request) and a full line (in a replay).
    */
    when (op_r.bits.requestType === TCacheRequestTypeEnum.WRITE) {
        // Chisel doesn't support slice assign so reshape first
        val data = Wire(Vec(mteTagsPerBlock, UInt(MTEConfig.tagBits.W)))
        data := resp_line.asTypeOf(data)
        data(op_r.bits.address.bits.addressMTETagIndex) := 
            op_r.bits.data(MTEConfig.tagBits, 0)
        resp_op_b.data := data.asUInt
    } .otherwise {
        resp_op_b.data := resp_line
    }

    /*
    Since an operation may be killed at any time, we need to invalidate the op
    but if we've already launched the request we need to be sure that we 
    correctly catch it so that we don't effectively have a use after free.
    */
    assert(!(resp_mem_v && io.req_op.ready), "Memory arrived while MSHR deallocated?")
    when (resp_mem_v) {
        printf("[mshr] fetched addr=%x, data=%x, request_alive=%d [tsc=%d]\n", op_address.bits.address, resp_line, op_r.valid, io.tsc_reg)
        resp_line_r := resp_mem_b.data
        resp_op_v_r := should_send_resp
        memory_executing := false.B
    }

    when (io.resp_op.fire) {
        resp_op_v_r := false.B
    }
}



/*
Notes:
    * The TCache should be livelock free since although misses are replayed,
      they cannot fail twice
    * Since we detect misses very quickly (available from S1), we can make the 
      pipeline stall free (and thus much less likely to deadlock) if we require
      at least two MSHRs. 
      If S1 missed, we need to have an MSHR available for it AND the incoming
      request we want to accept. If there is only one available, the request in
      S1 will take it, thereby leaving the incoming request with no MSHR
      available to it should it miss. Thus, we cannot accept it.
      If S1 hit, we can accept the incoming request so long as there is a free 
      MSHR.

      Having a stall free pipeline is really helpful too since it means that
      even with all MSHRs used we can still easily send line fills down the
      pipeline without any hacks since we're able to fairly directly guarantee
      forward progress.
*/