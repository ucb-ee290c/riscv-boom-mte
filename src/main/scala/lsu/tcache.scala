package lsu

import chisel3._
import chisel3.util._
import chisel3.util.random._
import freechips.rocketchip.config.{Parameters}
import boom.common._
import boom.lsu._
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
    blockSizeBytes:Int = 8,

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
    def cacheIndexOff = cacheOffsetOff
    def cacheTagBits = coreMaxAddrBits - log2Ceil(tcacheParams.blockSizeBytes)
    def cacheTagOff = cacheIndexOff
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

    /** 
     * Private request type; used when a miss to NLC returns and we want to
     * insert the new content into the cache.
     */
    val __FILL_REPLAY = Value
}

class NormalPhysicalAddress(implicit p: Parameters, tcacheParams: TCacheParams)
    extends TCacheBundle()(p, tcacheParams) {
    val address = UInt(width = coreMaxAddrBits.W)

    def getTagStoragePhysicalAddress(context:TCacheCoreIO) = {
        val tspa = Wire(Valid(new TagStoragePhysicalAddress()(p, tcacheParams)))

        val granuleAlignedAddress = address >> log2Ceil(MTEConfig.taggingGranuleBytes)
        tspa.bits.blockOffset := granuleAlignedAddress(mteTagsPerBlockBits - 1, 0)
        
        val enclosingRegionMap = VecInit(mteRegions.map {region =>
            address >= region.base.U && address < (region.base + region.size).U
        })
        /* 
        We're valid if we have one hit. It's a SoC design error to build 
        something with overlapping regions so we're also for sure 1H
        */
        tspa.valid := enclosingRegionMap.reduce( _ || _ )

        val regionPack = mteRegions zip (context.mteRegionBases zip context.mteRegionMasks)
        val tagStorageAddress = Mux1H(regionPack.map {case (region, (storageBase, storageMask)) =>
            val inBounds = address >= region.base.U && address < (region.base + region.size).U
            val regionOffset = granuleAlignedAddress - (region.base >> log2Ceil(MTEConfig.taggingGranuleBytes)).U
            
            val storageAddress = (storageBase + regionOffset) & storageMask
            inBounds -> storageAddress
        })

        tspa.bits.address := tagStorageAddress

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
    val requestType = TCacheRequestTypeEnum()
    val address = TagStoragePhysicalAddress()(p, tcacheParams)
    val data = UInt(width = tcacheParams.blockSizeBytes.W)
}

class TCacheLSUIO(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends TCacheBundle()(p, tcacheParams) {
    /** For sending requests to the TCache (read/write/flush) */
    val req = DecoupledIO(new TCacheRequest)
    /** For requests that require a response, this is where it comes out */
    val resp = Valid(new TCacheResponse)

    /** 
     * The TCache's access port (possibly arbitrated) to the next level 
     * Boom DCache
     */
    val nlc_req = Decoupled(new BoomDCacheReq)
    val nlc_resp = Valid(new BoomDCacheResp)
}

class TCacheCoreIO(implicit p: Parameters) extends BoomBundle()(p) {
    val mteRegionBases = Input(Vec(mteRegions.size, UInt(width = coreMaxAddrBits.W)))
    val mteRegionMasks = Input(Vec(mteRegions.size, UInt(width = coreMaxAddrBits.W)))
    /* The tag to return for memory not contained in any tag region */
    val mtePermissiveTag = Input(UInt(width = MTEConfig.tagBits.W))
}

class TCacheIO(implicit p: Parameters, tcacheParams: TCacheParams) 
    extends TCacheBundle()(p, tcacheParams) {
    /** IO bundle meant to be wielded by the LSU */
    val lsu:TCacheLSUIO = new TCacheLSUIO()(p, tcacheParams)
    val core:TCacheCoreIO = new TCacheCoreIO()(p)
}


class BoomNonBlockingTCacheModule(
    val tcacheParams: TCacheParams
)(implicit p: Parameters)
    extends TCacheModule()(p, tcacheParams) {
    require(tcacheParams.nMSHRs > 0, "nMSHRs must be non-zero")
    assert(isPow2(MTEConfig.tagBits), "TCache assumes an integer number of tags fit in a byte")
    require(tcacheParams.nSets * tcacheParams.blockSizeBytes <= dcacheParams.blockBytes, "L1D blocks must land in same L0T set")

    val io = IO(new TCacheIO()(p, tcacheParams))

    val dataArrays = DescribedSRAM(
        name = s"tcache_data",
        desc = "Non-blocking TCache Data Array",
        size = tcacheParams.nSets,
        data = Vec(tcacheParams.nWays, Bits((tcacheParams.blockSizeBytes * 8).W))
    )

    val metaT = new TCacheMetaEntry()(p, tcacheParams)
    val metaWidth = metaT.getWidth
    val metaArrays = DescribedSRAM(
        name = s"tcache_meta",
        desc = "Non-blocking TCache Metadata Array",
        size = tcacheParams.nSets,
        data = Vec(tcacheParams.nWays, Bits(metaWidth.W))
    )


    val s0_req = Wire(Valid(new TCacheRequest()(p, tcacheParams)))

    //TODO: Arbitrate between MSHR write requests
    s0_req.bits := io.lsu.req.bits
    s0_req.valid := io.lsu.req.valid

    /* Convert the incoming request to a tag storage address if needed */
    val s0_req_address = Wire(Valid(new TagStoragePhysicalAddress()(p, tcacheParams)))
    when (s0_req.bits.requestType === TCacheRequestTypeEnum.READ ||
            s0_req.bits.requestType === TCacheRequestTypeEnum.WRITE) {
        val normalAddress = Wire(new NormalPhysicalAddress()(p, tcacheParams))
        normalAddress.address := s0_req.bits.address
        s0_req_address := normalAddress.getTagStoragePhysicalAddress(io.core)
    } .otherwise {
        assert(s0_req.bits.requestType === TCacheRequestTypeEnum.FLUSH)
        /* Flushes come in with a pre translated address */
        val tagStorageAddress = Wire(Valid(new TagStoragePhysicalAddress()(p, tcacheParams)))
        tagStorageAddress.bits.address := s0_req.bits.address
         /* We're acting on a line(s) for flushes so this should never be used */
        tagStorageAddress.bits.blockOffset := DontCare
        //TODO: Revisit this. There are probably power gains to be had by
        //filtering flushes we know our cache won't hold
        tagStorageAddress.valid := true.B
        s0_req_address := tagStorageAddress
    }

    val s1_req = RegNext(s0_req)
    val s1_req_address = RegNext(s0_req_address)
    val s1_meta_read_value = Wire(Vec(tcacheParams.nWays, metaT))
    val s1_data_read_value = Wire(Vec(tcacheParams.nWays, Vec(mteTagsPerBlock, UInt(MTEConfig.tagBits.W))))
    /* We always need to access meta but don't always care to read data */
    s1_meta_read_value := metaArrays.read(
        s0_req_address.bits.addressIndex, 
        s0_req.valid && s0_req_address.valid
    ).map{ _.asTypeOf(metaT) }
    s1_data_read_value := dataArrays.read(
        s0_req_address.bits.addressIndex, 
        s0_req.valid && s0_req.bits.requestType === TCacheRequestTypeEnum.READ
    ).asTypeOf(s1_data_read_value)

    /* Hit all the ways */
    val s1_way_hit_map = VecInit(s1_meta_read_value.map {way_meta =>
        way_meta.cacheTag.valid && 
            way_meta.cacheTag.bits === s1_req_address.bits.addressTag
    })
    val s1_hit = s1_way_hit_map.reduce(_ || _)

    /* Stage 1: Read */
    /* Grab the read data for the hit way */
    val s1_read_hit_way_data = Mux1H(
        (s1_way_hit_map zip s1_data_read_value).map{ case (way_i_hit, way_data) =>
            way_i_hit -> way_data
        }
    )
    /* finally, extract the right offset (valid only if this is a read) */
    val s1_read_hit_data = s1_read_hit_way_data(s1_req_address.bits.blockOffset)

    /* Stage 1: Write */
    when (s1_req.valid && s1_req.bits.requestType === TCacheRequestTypeEnum.WRITE) {
        when (s1_hit) {
            /* Write hit! */
            //TODO: Arbitrate array write
            dataArrays.write(
                s1_req_address.bits.addressIndex, 
                VecInit(s1_way_hit_map.map{ hit =>
                    val result = Wire(UInt(width = MTEConfig.tagBits.W))
                    result := Mux(hit, s1_req.bits.data, DontCare)
                    result
                }),
                s1_way_hit_map
            )
        } .otherwise {
            /*
            We missed while trying to write. Start a miss.
            */
            //TODO!! Support write miss
        }
    }
    
    val s2_req = RegNext(s1_req)
    val s2_req_address = RegNext(s1_req_address)
    val s2_read_hit_data = RegNext(s1_read_hit_data)
    val s2_req_type = s2_req.bits.requestType
    val s2_hit = RegNext(s1_hit)

    /* Craft the response */
    val resp = io.lsu.resp
    resp.bits.uop := s2_req.bits.uop
    when (!s2_req.valid) {
        /* The request isn't valid, so we can't possibly have anything to say */
        resp.valid := false.B
        resp.bits := DontCare
    } .elsewhen(s2_req_type === TCacheRequestTypeEnum.READ) {
        when (!s2_req_address.valid) {
            /* 
            If the request is good but the address is not, this indicates that
            we launched a request for a region not covered by MTE. In this case,
            we simply return the permissive tag
            */
            resp.valid := true.B
            resp.bits.data := io.core.mtePermissiveTag
        } .otherwise {
            /*
            We had a valid address, so try to return the hit. If we missed,
            we'll push to the miss handler and get back to the requestor in a
            bit
            */
            resp.valid := s2_hit
            resp.bits.data := s2_read_hit_data
        }
    }

    // when (s0_req.bits.requestType === TCacheRequestTypeEnum.READ) {
    // } .elsewhen(s0_req.bits.requestType === TCacheRequestTypeEnum.WRITE) {

    // } .otherwise {
    //     assert(s0_req.bits.requestType === TCacheRequestTypeEnum.FLUSH)
    // }

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
    val req = DecoupledIO(new TCacheRequest)
    val resp = Valid(new TCacheRequest)
}

class TCacheMSHRModule(
    val tcacheParams: TCacheParams
)(implicit p: Parameters) extends TCacheModule()(p, tcacheParams) {

}