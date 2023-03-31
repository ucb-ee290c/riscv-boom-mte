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
    val cacheTag = Valid(UInt(width = cacheTagBits.W))
}

object TCacheRequestTypeEnum extends ChiselEnum {
    /** Read a 16-byte region's tag */
    val READ = Value

    /** 
     * Set a 16-byte region's tag. No response is generated; once a store 
     * is accepted by the cache, it will provide read forwarding as necessary
     */
    val WRITE = Value

    /** Evict and writeback a line */
    val FLUSH = Value
}

class NormalPhysicalAddress(implicit p: Parameters, tcacheParams: TCacheParams)
    extends TCacheBundle()(p, tcacheParams) {
    val address = UInt(width = coreMaxAddrBits.W)

    def getTagStoragePhysicalAddress(context:TCacheCoreIO) = {
        val tspa = Wire(Valid(new TagStoragePhysicalAddress()(p, tcacheParams)))

        val granuleAlignedAddress = address >> log2Ceil(MTEConfig.taggingGranuleBytes)
        tspa.bits.blockOffset := granuleAlignedAddress(mteTagsPerBlockBits - 1, 0)
        
        val regionPack = mteRegions zip (context.mteRegionBases zip context.mteRegionMasks)
        val tagStorageAddress = Mux1H(regionPack.map {case (region, (storageBase, storageMask)) =>
            val regionOffset = granuleAlignedAddress - (region.base >> log2Ceil(MTEConfig.taggingGranuleBytes)).U
            val storageAddress = storageBase + regionOffset
	    val storageAddressMasked = storageAddress & storageMask  
	    /* Is the target address contained by the region? */
            val inBounds = address >= region.base.U && address < (region.base + region.size).U && 
		/* Is the storage address in bounds? */
		storageAddress === storageAddressMasked &&
		/* Is the storage mask non-zero? A zero mask indicates a tagging is disabled for the region */
		storageMask =/= 0.U
	    val v = Wire(Valid(UInt(coreMaxAddrBits.W)))
	    v.valid := inBounds
	    v.bits := storageAddress
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
    val blockOffset = UInt(width = mteTagsPerBlockBits.W)
    
    def addressOffset = address(cacheOffsetOff + cacheOffsetBits - 1, cacheOffsetOff)
    def addressIndex = address(cacheIndexOff + cacheIndexBits - 1, cacheIndexOff)
    def addressTag = address(cacheTagOff + cacheTagBits - 1, cacheTagOff)
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
    /** If this is a read request, the requested tag, otherwise DontCare */
    val data = UInt(width = MTEConfig.tagBits.W)
}

class TCacheOperation(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends TCacheBundle()(p, tcacheParams) with HasBoomUOP {
    /** The kind of request this is */
    val requestType = TCacheRequestTypeEnum()
    /** Is this request being replayed due to a miss */
    val replay = Bool()
    /** The target, physical tag storage address this operation targets */
    val address = Valid(new TagStoragePhysicalAddress()(p, tcacheParams))
    /** 
     * The data for this operation. If this is a non-replayed write (i.e. the
     * first write), the low bits are the tag to write and the rest is DontCare.
     * If this is a replay due to the original operation missing, this is the
     * full line with the updated content (if needed, i.e. for a write).
     */
    val data = UInt(width = tcacheParams.blockSizeBytes.W)
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
    assert(isPow2(MTEConfig.tagBits), "TCache assumes an integer number of tags fit in a byte")
    require(tcacheParams.nSets * tcacheParams.blockSizeBytes <= dcacheParams.blockBytes, "L1D blocks must land in same L0T set")
    require(tcacheParams.blockSizeBytes <= coreDataBits / 8, "BOOM dcache does not currently provide transfers larger than coreDataBits")
    val io = IO(new TCacheIO()(p, tcacheParams))

    val mshrs = for (i <- 0 until tcacheParams.nMSHRs) yield {
        val mshr = Module(new TCacheMSHRModule(tcacheParams))
        val dcIF = Module(new SimpleHellaCacheIF)
        io.mem(i) <> dcIF.io.cache
        dcIF.io.requestor <> mshr.io.mem

        mshr
    }

    io.lsu.req.ready := true.B
    when (io.lsu.req.fire) {
        printf("[tcache] addr=0x%x, op=%d\n", io.lsu.req.bits.address, io.lsu.req.bits.requestType.asUInt)
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
        * Arbitrate, set meta address
    S1: 
        * Check for meta hit
        * If read, set data address based on hit 
        * If we missed meta, produce and enqueue miss request
    S2:
        * Mux out the tag result. The tag data will probably arrive late but
          it doesn't really matter since clients will probably be stuffing it in
          a register anyways (i.e. the LDQ/STQ)
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

    /* Decode the incoming request */
    val s0_incoming_req = io.lsu.req
    val s0_incoming_op = Wire(new TCacheOperation()(p, tcacheParams))
    val s0_incoming_op_valid = s0_incoming_req.valid
    s0_incoming_op.requestType := s0_incoming_req.bits.requestType
    s0_incoming_op.data := Cat(Fill(tcacheParams.blockSizeBytes - MTEConfig.tagBits, 0.U), s0_incoming_req.bits.data)
    s0_incoming_op.uop := s0_incoming_req.bits.uop
    s0_incoming_op.replay := false.B

    /* Convert the incoming request to a tag storage address if needed */
    when (s0_incoming_op.requestType === TCacheRequestTypeEnum.READ ||
            s0_incoming_op.requestType === TCacheRequestTypeEnum.WRITE) {
        val normalAddress = Wire(new NormalPhysicalAddress()(p, tcacheParams))
        normalAddress.address := s0_incoming_req.bits.address
        s0_incoming_op.address := normalAddress.getTagStoragePhysicalAddress(io.core)
    } .otherwise {
        assert(s0_incoming_op.requestType === TCacheRequestTypeEnum.FLUSH)
        /* Flushes come in with a pre translated address */
        val tagStorageAddress = Wire(Valid(new TagStoragePhysicalAddress()(p, tcacheParams)))
        tagStorageAddress.bits.address := s0_incoming_req.bits.address
         /* We're acting on a line(s) for flushes so this should never be used */
        tagStorageAddress.bits.blockOffset := DontCare
        //TODO: Revisit this. There are probably power gains to be had by
        //filtering flushes we know our cache won't hold
        tagStorageAddress.valid := true.B
        s0_incoming_op.address := tagStorageAddress
    }

    val s0_op = Wire(new TCacheOperation()(p, tcacheParams))
    val s0_op_valid = Wire(Bool())
    //TODO: Arbitrate between MSHR write requests
    s0_op := s0_incoming_op
    s0_op_valid := s0_incoming_op_valid

    val s1_op = RegNext(s0_op)
    val s1_op_valid = RegNext(s0_op_valid && 
        !IsKilledByBranch(io.core.brupdate, s0_op.uop)) &&
        !(io.core.exception && s0_op.requestType === TCacheRequestTypeEnum.READ)
    val s1_meta_read_value = Wire(Vec(tcacheParams.nWays, metaT))
    val s1_data_array_idx = Wire(UInt(dataArraysEntries.W))

    /* We always need to access meta but don't always care to read data */
    s1_meta_read_value := metaArrays.read(
        s0_op.address.bits.addressIndex, 
        s0_op_valid && s0_op.address.valid
    ).map{ _.asTypeOf(metaT) }

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

    s1_data_array_idx := s1_op.address.bits.addressIndex << nWaysBits | s1_way_hit_idx

    // /* Stage 1: Read */
    // /* Grab the read data for the hit way */
    // val s1_read_hit_way_data = Mux1H(
    //     (s1_way_hit_map zip s1_data_read_value).map{ case (way_i_hit, way_data) =>
    //         way_i_hit -> way_data
    //     }
    // )
    // /* finally, extract the right offset (valid only if this is a read) */
    // val s1_read_hit_data = s1_read_hit_way_data(s1_op.bits.address.bits.blockOffset)

    // /* Stage 1: Write */
    // when (s1_op.valid && s1_op.bits.requestType === TCacheRequestTypeEnum.WRITE) {
    //     when (s1_hit) {
    //         /* Write hit! */
    //         //TODO: Arbitrate array write
    //         dataArrays.write(
    //             s1_op.bits.address.bits.addressIndex, 
    //             VecInit(s1_way_hit_map.map{ hit =>
    //                 val result = Wire(UInt(width = MTEConfig.tagBits.W))
    //                 result := Mux(hit, s1_op.bits.data(MTEConfig.tagBits - 1, 0), DontCare)
    //                 result
    //             }),
    //             s1_way_hit_map
    //         )
    //     } .otherwise {
    //         /*
    //         We missed while trying to write. Start a miss.
    //         */
    //         //TODO!! Support write miss
    //     }
    // }
    
    val s2_op = RegNext(s1_op)
    val s2_op_valid = RegNext(s1_op_valid && 
        !IsKilledByBranch(io.core.brupdate, s1_op.uop)) &&
        !(io.core.exception && s1_op.requestType === TCacheRequestTypeEnum.READ)
    val s2_data_read_value = Wire(Vec(mteTagsPerBlock, UInt(MTEConfig.tagBits.W)))
    val s2_op_type = s2_op.requestType
    val s2_hit = RegNext(s1_hit)

    s2_data_read_value := dataArrays.read(
        s1_data_array_idx, 
        s1_op_valid && s1_op.requestType === TCacheRequestTypeEnum.READ && 
            s1_op.address.valid && s1_hit
    )

    /* Craft the response */
    val resp = io.lsu.resp
    resp.bits.uop := s2_op.uop
    when (!s2_op_valid) {
        /* The request isn't valid, so we can't possibly have anything to say */
        resp.valid := false.B
        resp.bits := DontCare
    } .elsewhen(s2_op_type === TCacheRequestTypeEnum.READ) {
        when (!s2_op.address.valid) {
            /* 
            If the request is good but the address is not, this indicates that
            we launched a request for a region not covered by MTE. In this case,
            we simply return the permissive tag
            */
	        printf("[tache] resp default = uses_ldq=%d, ldq=%d, uses_stq=%d, stq=%d\n", s2_op.uop.uses_ldq, s2_op.uop.ldq_idx, s2_op.uop.uses_stq, s2_op.uop.stq_idx) 
            resp.valid := true.B
            resp.bits.data := io.core.mtePermissiveTag
        } .otherwise {
            /*
            We had a valid address, so try to return the hit. If we missed,
            we'll push to the miss handler and get back to the requestor in a
            bit
            */
            resp.valid := s2_hit
            resp.bits.data := s2_data_read_value(s2_op.address.bits.blockOffset)
            assert(false.B, "not impl")
        }
    } .otherwise {
    	assert(false.B, "TCache unexpected s2_op type")
    }

    // when (s0_req.bits.requestType === TCacheRequestTypeEnum.READ) {
    // } .elsewhen(s0_req.bits.requestType === TCacheRequestTypeEnum.WRITE) {

    // } .otherwise {
    //     assert(s0_req.bits.requestType === TCacheRequestTypeEnum.FLUSH)
    // }

    mshrs.foreach { mshr =>
        mshr.io.req_op.valid := false.B
        mshr.io.req_op.bits := DontCare
    }

    /*
    Read:
    * s0: set index
    * s1: nop
    * s2
    * Combinatorially set the address, clock edge launches read
    * Buffer the read since it'll



    The cache is pipelined into three stages
    1. Index meta and data by set
    2. Way select data based on meta
    3. Response
    */
}

class TCacheMSHRIO(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends TCacheBundle()(p, tcacheParams) {
    /** 
     * The operation that missed. This operation will be fulfilled and passed
     * back on resp_op to be replayed
     */
    val req_op = Flipped(DecoupledIO(new TCacheOperation))
    /** When valid, the completed miss ready to be directly replayed. */
    val resp_op = Valid(new TCacheOperation)

    /** This misshandler's request to and response from NLC */
    val mem = new HellaCacheIO
}

class TCacheMSHRModule(
    val tcacheParams: TCacheParams
)(implicit p: Parameters) extends TCacheModule()(p, tcacheParams) {
    val io = IO(new TCacheMSHRIO()(p, tcacheParams))
    
    val op = RegInit({
        val op_i = Wire(Valid(new TCacheOperation()(p, tcacheParams)))
        op_i.valid := false.B
        op_i.bits := DontCare
        op_i
    })

    /* We can accept a new request when we're done with the one we have */
    io.req_op.ready := !op.valid
    when (io.req_op.fire) {
    	assert(io.req_op.valid, "MSHR fired on an invalid op?")
        op.bits := io.req_op.bits
        op.valid := true.B
    }
    
    //TODO: Support brupdate
    //TODO: Support writes
    val op_valid = Mux(io.req_op.fire, io.req_op.valid, op.valid)
    val op_address = Mux(io.req_op.fire, io.req_op.bits.address, op.bits.address)
    val op_type = Mux(io.req_op.fire, io.req_op.bits.requestType, op.bits.requestType)

    when (op_valid) {
        when (op_type === TCacheRequestTypeEnum.READ) {
            /* Read miss */
	    //io.mem.req
        }
    }
    io.mem := DontCare
    io.mem.req.valid := false.B

    io.resp_op.valid := false.B
    io.resp_op.bits := DontCare
}



/*
Notes:
    * The TCache should be livelock free since although misses are replayed,
      they cannot fail twice
*/
